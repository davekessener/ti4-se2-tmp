#include <algorithm>
#include <stdarg.h>
#include <stdio.h>

#include "lib/Timer.h"
#include "lib/log/Logger.h"

#define MXT_BUFSIZE 4096

namespace lib { namespace log {

// # ---------------------------------------------------------------------------

void Logger::addParent(Logger_ptr p)
{
	Lock guard(this);

	parents_.push_back(p);
}

void Logger::removeParent(Logger_ptr p)
{
	Lock guard(this);

	parents_.erase(std::find(parents_.begin(), parents_.end(), p));
}

void Logger::addHandler(Handler_ptr p)
{
	Lock guard(this);

	handlers_.push_back(p);
}

void Logger::removeHandler(Handler_ptr p)
{
	Lock guard(this);

	handlers_.erase(std::find(handlers_.begin(), handlers_.end(), p));
}

void Logger::addFilter(Filter_ptr p)
{
	Lock guard(this);

	filters_.push_back(p);
}

void Logger::removeFilter(Filter_ptr p)
{
	Lock guard(this);

	filters_.erase(std::find(filters_.begin(), filters_.end(), p));
}

// # ---------------------------------------------------------------------------

void Logger::log(const LogLevel& ll, const std::string& format, const char *s, int l, ...)
{
	char buf[MXT_BUFSIZE];
	va_list vl;

	va_start(vl, l);
	vsnprintf(buf, MXT_BUFSIZE, format.c_str(), vl);
	va_end(vl);
	buf[MXT_BUFSIZE-1] = '\0';

	log(LogRecord(ll, Timer::timestamp(), ThreadManager::instance().getCurrent(), std::string(buf), s, l));
}

void Logger::log(const LogRecord& lr)
{
	Lock guard(this);

	for(fiter_t i1 = filters_.begin(), i2 = filters_.end() ; i1 != i2 ; ++i1)
	{
		if(!(*i1)->accept(lr)) return;
	}

	for(hiter_t i1 = handlers_.begin(), i2 = handlers_.end() ; i1 != i2 ; ++i1)
	{
		(*i1)->handle(lr);
	}

	for(piter_t i1 = parents_.begin(), i2 = parents_.end() ; i1 != i2 ; ++i1)
	{
		(*i1)->log(lr);
	}
}

}}

