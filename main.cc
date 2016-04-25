#include "lib/log/LogManager.h"

int main(int argc, char *argv[])
{
	using lib::log::Logger_ptr;
	using lib::log::LogManager;

	Logger_ptr log = LogManager::instance().rootLog();

	log->MXT_LOG("Hello, World!");

	return 0;
}

