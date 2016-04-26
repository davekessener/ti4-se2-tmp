#include <string>

#include "lib/log/LogManager.h"
#include "lib/TimeP.h"
#include "lib/Data.h"

#include "Connection.h"

// # ===========================================================================

using lib::log::Logger_ptr;
using lib::log::LogManager;
using lib::log::LogLevel;
using lib::Time;
using lib::Data_ptr;
using lib::Data;

Logger_ptr getLog()
{
	static Logger_ptr log = LogManager::instance().rootLog();

	return log;
}

// # ===========================================================================

using hw::Connection;

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

