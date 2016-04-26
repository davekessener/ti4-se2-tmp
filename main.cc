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
#include "lib/RingBuffer.hpp"
#include "lib/Data.h"

// # ===========================================================================

#define MXT_IOSPEED B19200
#define MXT_CONLOG "conlog"

#define TOK_A   0x8de29457
#define TOK_OK  0x226442b8
#define TOK_HSA 0x88e7d7c9
#define TOK_HSB 0x2624d967
#define TOK_DAT 0x36773fe0

using lib::log::Logger_ptr;
using lib::log::LogManager;
using lib::log::LogLevel;
using lib::Time;
using lib::Thread;
using lib::Data_ptr;
using lib::Data;

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

#define MXT_PACKETBUFSIZE 256

class Connection
{
	typedef lib::RingBuffer<Data_ptr, MXT_PACKETBUFSIZE, lib::RingBufferConcurrency::MultiThreaded> buf_t;

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
		bool doneWriting(void) const { return wBuf_.empty(); }

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

		struct Packet
		{
			explicit Packet(uint32_t t) : tag(t) { }
			Packet(uint32_t t, Data_ptr p) : tag(t), data(p) { }

			uint32_t tag;
			Data_ptr data;
		};

		void sendPacket(const Packet& p)
		{
			send<uint32_t>(p.tag);
			if(static_cast<bool>(p.data) && p.data->size() > 0)
			{
				send<uint32_t>(p.data->size());
				send(p.data->data(), p.data->size());
			}
			else
			{
				send<uint32_t>(0);
			}
		}

		Packet receivePacket(void)
		{
			Packet p(recv<uint32_t>());
			uint32_t size = recv<uint32_t>();

			if(size > 0)
			{
				p.data = Data::empty(size);
				recv(p.data->data(), size);
			}

			return p;
		}

		void checkedSend(const Packet& p)
		{
			sendPacket(p);
			Packet a = receivePacket();
			if(a.tag != TOK_OK) throw std::string("invalid answer!");
		}

		Packet checkedRecv(void)
		{
			Packet p = receivePacket();
			sendPacket(Packet(TOK_OK));
			return p;
		}

// # ---------------------------------------------------------------------------

		void flush(void)
		{
			if(tcflush(f_, TCIOFLUSH) == -1)
				throw std::string("failed flush");
		}

// # ---------------------------------------------------------------------------

		void giveHandshake(void)
		{
			uint32_t r = 0;

			while(true)
			{
				send<uint32_t>(TOK_HSA);

				Time::ms(100).wait();
				
				if(try_recv(&r, sizeof(r)))
				{
					if(r == TOK_HSB)
						break;
					else
						throw std::string("failed hand shake");
				}
			}
		}

		void receiveHandshake(void)
		{
			uint32_t r = recv<uint32_t>();

			if(r != TOK_HSA)
			{
				getLog()->MXT_LOGL(LogLevel::ERROR, "failed hand shake: 0x%08x", r);
				throw std::string("failed hand shake");
			}

			send<uint32_t>(TOK_HSB);

			flush();
		}

// # ---------------------------------------------------------------------------

		void sendData(void)
		{
			while(!wBuf_.empty())
			{
				checkedSend(Packet(TOK_DAT, wBuf_.dequeue()));
				getLog()->MXT_LOG("send data packet");
			}
			checkedSend(Packet(TOK_A));
		}

		void recvData(void)
		{
			while(true)
			{
				Packet a = checkedRecv();

				if(a.tag == TOK_A)
					break;

				getLog()->MXT_LOG("received data packet");

				rBuf_.enqueue(a.data);
			}
		}

// # ---------------------------------------------------------------------------
		
		void sendData(Data_ptr p)
		{
			wBuf_.enqueue(p);
		}

		Data_ptr receiveData(void)
		{
			return rBuf_.dequeue();
		}

		bool hasData(void) const
		{
			return !rBuf_.empty();
		}

// # ---------------------------------------------------------------------------
		
		void run(void)
		{
			running_ = true;

			try
			{
				if(active_)
					giveHandshake();
				else
					receiveHandshake();

				connected_ = true;

				while(true)
				{
					if(active_)
					{
						sendData();
						active_ = false;
					}
					else
					{
						recvData();
						active_ = true;
					}

					getLog()->MXT_LOG("switched, now %s", (active_ ? "ACTIVE" : "PASSIVE"));

					checkRunning();

					Time::ms(100).wait();
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
		buf_t rBuf_, wBuf_;
};

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
		std::string msg("Hello, World!");

		while(!c.connected()) Time::ms(100).wait();

		if(active) c.sendData(Data::get(msg.c_str(), msg.length() + 1));
		Data_ptr p = c.receiveData();
		if(!active) c.sendData(p);

		log->MXT_LOG("received string \"%s\"", (const char *) p->data());

		while(!c.doneWriting()) Time::ms(100).wait();

		c.close();
	}
	catch(const std::string& e)
	{
		log->MXT_LOGL(LogLevel::ERROR, "%s [errno %i (%s)]", e.c_str(), errno, strerror(errno));
	}

	return 0;
}

