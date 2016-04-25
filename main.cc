#include <string>

#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>

#include "lib/log/LogManager.h"
#include "lib/TimeP.h"

// # ===========================================================================

#define MXT_IOSPEED B19200

#define MXT_CONLOG "conlog"

using lib::log::Logger_ptr;
using lib::log::LogManager;
using lib::log::LogLevel;
using lib::Time;

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
		int r = read(f, pp, n);

		if(r == -1)
		{
			if(errno != EAGAIN && errno != EWOULDBLOCK)
				throw std::string("failed read");
			else
				r = 0;
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
		Connection(const std::string& d)
		{
			if((f_ = open(d.c_str(), O_RDWR | O_NOCTTY | O_NDELAY)) < 0)
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
		}

		~Connection( )
		{
			close(f_);
		}

		void send(const void *pp, size_t n)
		{
			size_t t = 0;
			int r = 0;
			const uint8_t *p = (const uint8_t *) pp;

			log_->MXT_LOG("BEGIN writing %u bytes ...", n);

			while(t < n)
			{
				r = nb_write(f_, p + t, n - t);

				for(int i = 0 ; i < r ; ++i)
				{
					log_->MXT_LOG("wrote 0x%02x", (unsigned)p[t+i]);
				}

				t += r;

				log_->MXT_LOG("wrote %i bytes", r);
			}

			log_->MXT_LOG("TOTAL of %u bytes written", t);
		}

		bool try_recv(void *pp, size_t n)
		{
			size_t t = 0;
			int r = 0;
			uint8_t *p = (uint8_t *) pp;

			log_->MXT_LOG("BEGIN reading %u bytes ...", n);

			while(t < n)
			{
				r = nb_read(f_, p + t, n - t);

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

		void recv(void *pp, size_t n)
		{
			while(!try_recv(pp, n));
		}

	private:
		int f_;
		Logger_ptr log_;
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
		Connection c("/dev/ttyS0");

		if(active)
		{
			sendS(c, "Hello, World!");
		}

		std::string s = recvS(c);

		if(!active)
		{
			sendS(c, s);
		}

		log->MXT_LOG("received string \"%s\"", s.c_str());
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

