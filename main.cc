#include <string>

#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>

#include "lib/log/LogManager.h"
#include "test/SerialTest.h"

using lib::log::Logger_ptr;
using lib::log::LogManager;
using lib::log::LogLevel;

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
			if((f_ = open(d.c_str(), O_RDWR)) < 0)
				throw std::string("couldn't open device '" + d + "'!");

			struct termios ts;
			tcflush(f_, TCIOFLUSH);
			tcgetattr(f_, &ts);
			cfsetispeed(&ts, B19200);
			cfsetospeed(&ts, B19200);
			ts.c_cflag &= ~CSIZE;
			ts.c_cflag &= ~CSTOPB;
			ts.c_cflag &= ~PARENB;
			ts.c_cflag |= CS8;
			ts.c_cflag |= CREAD;
			ts.c_cflag |= CLOCAL;
			tcsetattr(f_, TCSANOW, &ts);
		}

		~Connection( )
		{
			close(f_);
		}

		void send(const void *pp, size_t n)
		{
			size_t t = 0;
			int r = 0;
			const char *p = (const char *) pp;

			Logger_ptr log = getLog();

			log->MXT_LOG("BEGIN writing %u bytes ...", n);

			while(t < n)
			{
				if((r = write(f_, p + t, n - t)) == -1)
					throw std::string("failed send");
				t += r;

				log->MXT_LOG("wrote %i bytes", r);
			}

			log->MXT_LOG("TOTAL of %u bytes written", t);
		}

		void recv(void *pp, size_t n)
		{
			size_t t = 0;
			int r = 0;
			char *p = (char *) pp;

			Logger_ptr log = getLog();

			log->MXT_LOG("BEGIN reading %u bytes ...", n);

			while(t < n)
			{
				if((r = read(f_, p + t, n - t)) == -1)
					throw std::string("failed recv");
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
	test::basicSerialA();
#else
	test::basicSerialB();
#endif

	return 0;
}

int o_main(int argc, char *argv[])
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
			c.sendS("Hello, World!");
		}

		std::string s = c.recvS();

		if(!active)
		{
			c.sendS(s);
		}

		log->MXT_LOG("recv string \"%s\"", s.c_str());
	}
	catch(const std::string& e)
	{
		log->MXT_LOGL(LogLevel::ERROR, "%s [errno %i (%s)]", e.c_str(), errno, strerror(errno));
	}

	return 0;
}

