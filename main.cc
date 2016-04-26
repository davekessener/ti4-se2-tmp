#include <string>
#include <memory>

#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <sys/time.h>
#include <sys/types.h>

#include "lib/log/LogManager.h"
#include "lib/TimeP.h"
#include "lib/mpl/FtorWrapper.hpp"
#include "lib/concurrent/Thread.h"

// # ===========================================================================

#define MXT_IOSPEED B19200
#define MXT_CONLOG "conlog"

#define TOK_A   0x8de29457
#define TOK_OK  0x226442b8
#define TOK_HSA 0x88e7d7c9
#define TOK_HSB 0x2624d967
#define TOK_Q   0x36773fe0

using lib::log::Logger_ptr;
using lib::log::LogManager;
using lib::log::LogLevel;
using lib::Time;
using lib::Thread;

Logger_ptr getLog()
{
	static Logger_ptr log = LogManager::instance().rootLog();

	return log;
}

// # ===========================================================================

namespace
{
	int nb_read(int f, void *pp, size_t n)
	{
		fd_set rfds;
		struct timeval tv;

		FD_ZERO(&rfds);
		FD_SET(f, &rfds);

		tv.tv_sec = 0;
		tv.tv_usec = 0;

		int r = select(f + 1, &rfds, NULL, NULL, &tv);

		if(r < 0)
			throw std::string("failed to check for readability");

		if(r > 0)
		{
			r = read(f, pp, n);

			if(r == -1)
			{
				if(errno != EAGAIN && errno != EWOULDBLOCK)
					throw std::string("failed read");
				else
					r = 0;
			}
		}

		return r;
	}

	int nb_write(int f, const void *pp, size_t n)
	{
		int r = write(f, pp, n);

		if(r == -1)
		{
			if(errno != EAGAIN && errno != EWOULDBLOCK)
				throw std::string("failed write");
			else
				r = 0;
		}

		return r;
	}
}

// # ===========================================================================

class Connection
{
	public:
		Connection(const std::string& d, bool a)
		: device_(d), active_(a), running_(false), connected_(false)
		{
			if((f_ = open(d.c_str(), O_RDWR | O_NOCTTY)) < 0)
				throw std::string("couldn't open device '" + d + "'!");

			if(fcntl(f_, F_SETFL, 0) < 0) throw std::string("fcntl");

			struct termios ts;
			if(tcflush(f_, TCIOFLUSH) < 0) throw std::string("tcflush");
			if(tcgetattr(f_, &ts) < 0) throw std::string("tcgetattr");
			if(cfsetispeed(&ts, MXT_IOSPEED) < 0) throw std::string("cfsetispeed");
			if(cfsetospeed(&ts, MXT_IOSPEED) < 0) throw std::string("cfsetospeed");
			ts.c_cflag &= ~CSIZE;
			ts.c_cflag &= ~CSTOPB;
			ts.c_cflag &= ~PARENB;
			ts.c_cflag |= CS8;
			ts.c_cflag |= CREAD;
			ts.c_cflag |= CLOCAL;

			ts.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
			ts.c_iflag &= ~(IXON | IXOFF | IXANY);
			ts.c_cflag &= ~CRTSCTS;
			ts.c_oflag &= ~OPOST;

			ts.c_oflag &= ~(ONLCR | OCRNL);
			ts.c_iflag &= ~(INLCR | ICRNL);

			if(tcsetattr(f_, TCSANOW, &ts) < 0) throw std::string("tcsetattr");

			log_ = LogManager::instance().getLog(MXT_CONLOG);

//			log_->addParent(getLog());

			thread_.reset(new Thread(lib::wrapInFtor(this, &Connection::run)));
		}

// # ---------------------------------------------------------------------------

		~Connection( )
		{
			close();
		}

		void close(void)
		{
			if(f_)
			{
				running_ = false;
				thread_->join();
				thread_.reset();
				::close(f_);
				f_ = 0;
			}
		}

		bool connected(void) const { return connected_; }
		bool running(void) const { return running_; }

// # ---------------------------------------------------------------------------

		void send(const void *pp, size_t n)
		{
			size_t t = 0;
			int r = 0;
			const uint8_t *p = (const uint8_t *) pp;

			log_->MXT_LOG("BEGIN writing %u bytes ...", n);

			while(t < n)
			{
				r = nb_write(f_, p + t, n - t);

				checkRunning();

				for(int i = 0 ; i < r ; ++i)
				{
					log_->MXT_LOG("wrote 0x%02x", (unsigned)p[t+i]);
				}

				t += r;

				log_->MXT_LOG("wrote %i bytes", r);
			}

			log_->MXT_LOG("TOTAL of %u bytes written", t);
		}

// # ---------------------------------------------------------------------------

		bool try_recv(void *pp, size_t n)
		{
			size_t t = 0;
			int r = 0;
			uint8_t *p = (uint8_t *) pp;

			log_->MXT_LOG("BEGIN reading %u bytes ...", n);

			while(t < n)
			{
				r = nb_read(f_, p + t, n - t);

				checkRunning();

				if(!r && !t) return false;

				for(int i = 0 ; i < r ; ++i)
				{
					log_->MXT_LOG("read 0x%02x", (unsigned)p[t+i]);
				}

				t += r;

				log_->MXT_LOG("read %i bytes", r);
			}

			log_->MXT_LOG("TOTAL of %u bytes read", t);

			return true;
		}

// # ---------------------------------------------------------------------------

		void recv(void *pp, size_t n)
		{
			while(!try_recv(pp, n)) checkRunning();
		}

// # ---------------------------------------------------------------------------
	
		template<typename T>
		void send(const T& t)
		{
			send(&t, sizeof(t));
		}

		template<typename T>
		T recv(void)
		{
			T t;
			recv(&t, sizeof(t));
			return t;
		}

// # ---------------------------------------------------------------------------

		struct DoneRunning { };
	
		void checkRunning(void)
		{
			if(!running_)
				throw DoneRunning();
		}

// # ---------------------------------------------------------------------------

		void checkedSend(uint32_t tok)
		{
			send<uint32_t>(tok);
			uint32_t a = recv<uint32_t>();
			if(a != TOK_OK) throw std::string("invalid answer!");
		}

		uint32_t checkedRecv(void)
		{
			uint32_t a = recv<uint32_t>();
			send<uint32_t>(TOK_OK);
			return a;
		}

		void flush(void)
		{
			if(tcflush(f_, TCIOFLUSH) == -1)
				throw std::string("failed flush");
		}

// # ---------------------------------------------------------------------------
		
		void run(void)
		{
			Time delay = Time::ms(100);

			running_ = true;

			try
			{
				if(active_)
				{
					uint32_t r = 0;

					while(true)
					{
						send<uint32_t>(TOK_HSA);

						delay.wait();
						
						if(try_recv(&r, sizeof(r)))
						{
							if(r == TOK_HSB)
								break;
							else
								throw std::string("failed hand shake");
						}
					}
				}
				else
				{
					uint32_t r = recv<uint32_t>();

					if(r != TOK_HSA)
						throw std::string("failed hand shake");

					send<uint32_t>(TOK_HSB);

					flush();
				}

				connected_ = true;

				while(running_)
				{
					if(active_)
					{
						checkedSend(TOK_A);
						active_ = false;
					}
					else
					{
						uint32_t a = checkedRecv();
						if(a != TOK_A)
						{
							getLog()->MXT_LOGL(LogLevel::ERROR, "invalid A token [0x%08x instead of 0x%08x]", a, TOK_A);
							throw std::string("invalid A token");
						}
						active_ = true;
					}

					getLog()->MXT_LOG("switched, now %s", (active_ ? "ACTIVE" : "PASSIVE"));

					delay.wait();
				}
			}
			catch(const DoneRunning& e)
			{
				getLog()->MXT_LOG("shutting down connection");
			}
			catch(const std::string& e)
			{
				getLog()->MXT_LOG("caught exception: \"%s\" [errno %i (%s)]", e.c_str(), errno, strerror(errno));
				throw;
			}

			connected_ = false;
		}

// # ---------------------------------------------------------------------------

	private:
		std::string device_;
		bool active_, running_, connected_;
		int f_;
		Logger_ptr log_;
		std::auto_ptr<Thread> thread_;
};

// # ===========================================================================

void sendS(Connection&, const std::string&);
std::string recvS(Connection&);

// # ===========================================================================

int main(int argc, char *argv[])
{
#ifdef ACTIVE
	static const bool active = true;
#else
	static const bool active = false;
#endif

	Logger_ptr log = getLog();

	log->MXT_LOG("Starting serial test as %s", (active ? "ACTIVE" : "PASSIVE"));

	try
	{
		Connection c("/dev/ttyS0", active);

		while(!c.connected()) Time::ms(100).wait();

		Time::s(1).wait();

		c.close();
	}
	catch(const std::string& e)
	{
		log->MXT_LOGL(LogLevel::ERROR, "%s [errno %i (%s)]", e.c_str(), errno, strerror(errno));
	}

	return 0;
}

// # ===========================================================================

void sendS(Connection& c, const std::string& s)
{
	uint16_t l = s.length();

	c.send(&l, sizeof(l));
	c.send(s.c_str(), l);
}

std::string recvS(Connection& c)
{
	uint16_t l;
	char *buf;
	std::string s;

	c.recv(&l, sizeof(l));
	buf = new char[l + 1];
	c.recv(buf, l);
	buf[l] = '\0';

	s = buf;
	delete[] buf;

	return s;
}

