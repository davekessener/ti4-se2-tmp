#include <string>

#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>

#include "lib/log/LogManager.h"
#include "lib/TimeP.h"

//#define MXT_IOSPEED B19200
#define MXT_IOSPEED B115200

using lib::log::Logger_ptr;
using lib::log::LogManager;
using lib::log::LogLevel;
using lib::Time;

Logger_ptr getLog()
{
	static Logger_ptr log = LogManager::instance().rootLog();

	return log;
}

class Connection
{
	public:
		Connection(const std::string& d)
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

			if(tcsetattr(f_, TCSANOW, &ts) < 0) throw std::string("tcsetattr");
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

			Logger_ptr log = getLog();

			log->MXT_LOG("BEGIN writing %u bytes ...", n);

			while(t < n)
			{
				if((r = write(f_, p + t, n - t)) == -1)
					throw std::string("failed send");

				for(int i = 0 ; i < r ; ++i)
				{
					log->MXT_LOG("wrote 0x%02x", (unsigned)p[t+i]);
				}

				t += r;

				log->MXT_LOG("wrote %i bytes", r);
			}

			log->MXT_LOG("TOTAL of %u bytes written", t);
		}

		void recv(void *pp, size_t n)
		{
			size_t t = 0;
			int r = 0;
			uint8_t *p = (uint8_t *) pp;

			Logger_ptr log = getLog();

			log->MXT_LOG("BEGIN reading %u bytes ...", n);

			while(t < n)
			{
				if((r = read(f_, p + t, n - t)) == -1)
					throw std::string("failed recv");

				for(int i = 0 ; i < r ; ++i)
				{
					log->MXT_LOG("read 0x%02x", (unsigned)p[t+i]);
				}

				t += r;

				log->MXT_LOG("read %i bytes", r);
			}

			log->MXT_LOG("TOTAL of %u bytes read", t);
		}

		void sendS(const std::string& s)
		{
			uint16_t l = s.length();

			send(&l, sizeof(l));
			send(s.c_str(), l);

			getLog()->MXT_LOG("wrote string of size %u", (unsigned) l);
		}

		std::string recvS(void)
		{
			uint16_t l;
			char *buf;
			std::string s;

			recv(&l, sizeof(l));
			getLog()->MXT_LOG("will read a string of size %u", (unsigned) l);
			buf = new char[l + 1];
			recv(buf, l);
			buf[l] = '\0';

			s = buf;
			delete[] buf;

			getLog()->MXT_LOG("read string of size %u", (unsigned) l);

			return s;
		}

	private:
		int f_;
};

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
		static const uint64_t tt = 0x120d00789abcdef0L;

		if(active)
		{
			c.send(&tt, sizeof(tt));
			c.sendS("Hello, World");
		}
		else
		{
			uint64_t rt = -1;
			c.recv(&rt, sizeof(rt));
			log->MXT_LOG("MAGIC: 0x%08x%08x", ((uint32_t *) &rt)[1], ((uint32_t *) &rt)[0]);

			std::string s = c.recvS();
			log->MXT_LOG("received \"%s\"", s.c_str());
		}

		Time::ms(100).wait();

//		if(active)
//		{
//			c.sendS("Hello, World!");
//		}
//
//		std::string s = c.recvS();
//
//		if(!active)
//		{
//			c.sendS(s);
//		}
//
//		log->MXT_LOG("recv string \"%s\"", s.c_str());
	}
	catch(const std::string& e)
	{
		log->MXT_LOGL(LogLevel::ERROR, "%s [errno %i (%s)]", e.c_str(), errno, strerror(errno));
	}

	return 0;
}

