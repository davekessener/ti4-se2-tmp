#include <fcntl.h>
#include <termios.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <stdexcept>
#include <iostream>

#include "test/SerialTest.h"
#include "lib/TimeP.h"
#include "lib/log/LogManager.h"

namespace test
{
	int writeW(int f, const void *d, size_t s)
	{
		int r = write(f, d, s);
		if(r < 0)
			MXT_TODO_ERROR;
		return r;
	}

	int readcondW(int f, void *d, size_t s, size_t m, int t, int tt)
	{
		int r = read(f, d, s);
		if(r < 0)
			MXT_TODO_ERROR;
		return r;
	}

	int writeS(int f, const std::string& s)
	{
		unsigned char l = s.length();
		int r = 0;
		unsigned t;

		t = 0;
		while((t += writeW(f, &l, sizeof(l))) < sizeof(l));

		r += t;
		t = 0;
		while((t += writeW(f, s.c_str() + t, l - t)) < l);

		r += t;

		return r;
	}

	std::string readS(int f, int *rr)
	{
		unsigned char l;
		int r = 0;
		unsigned t;
		char *buf = NULL;

		t = 0;
		while((t += readcondW(f, &l, sizeof(l), 0, 0, 0)) < sizeof(l));

		buf = new char[l + 1];

		r += t;
		t = 0;
		while((t += readcondW(f, buf + t, l - t, 0, 0, 0)) < l);

		r += t;
		*rr = r;
		buf[l] = '\0';

		std::string s(buf);
		delete[] buf;
		return s;
	}

	void basicSerial(bool active)
	{
		lib::log::Logger_ptr log = lib::log::LogManager::instance().rootLog();

		log->MXT_LOG("enter serial test %s", (active ? "ACTIVE" : "PASSIVE"));

		int f;
		if((f = open("/dev/ttyS0", O_RDWR)) < 0)
			throw std::runtime_error("err establishing serial connection");

		struct termios ts;
		tcflush(f, TCIOFLUSH);
		tcgetattr(f, &ts);
		cfsetispeed(&ts, B19200);
		cfsetospeed(&ts, B19200);
		ts.c_cflag &= ~CSIZE;
		ts.c_cflag &= ~CSTOPB;
		ts.c_cflag &= ~PARENB;
		ts.c_cflag |= CS8;
		ts.c_cflag |= CREAD;
		ts.c_cflag |= CLOCAL;
		tcsetattr(f, TCSANOW, &ts);

		int w = 0;

		if(active)
		{
			w = writeS(f, "Hello, World!");
		}

		int r;
		std::string s = readS(f, &r);

		if(!active)
		{
			w = writeS(f, s);
		}

		log->MXT_LOG("read \"%s\" (%i -> %i)", s.c_str(), r, w);

		close(f);
	}

	void basicSerialA(void) { basicSerial(true); }
	void basicSerialB(void) { basicSerial(false); }
}
