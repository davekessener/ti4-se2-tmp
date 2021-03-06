#ifndef DAV_LIB_TEST_UNITTEST_H
#define DAV_LIB_TEST_UNITTEST_H

#include <test/TestManager.h>
#include <test/UnitTest.h>
#include <macro.h>

// # ---------------------------------------------------------------------------

#define BEGIN \
namespace TEST_CASE_NAME { \
struct Base { }; \
static ::lib::test::TestManager::Selector select_unit_##TEST_CASE_NAME(MXT_STRINGIFY(TEST_CASE_NAME)); \
template<typename T> struct Data { }; \
template<typename T> struct Before : public virtual Data<T> { }; \
template<typename T> struct After : public virtual Data<T> { };

// # ---------------------------------------------------------------------------

#define END \
}

// # ---------------------------------------------------------------------------

#define SUPER(super_class) \
template<> \
struct Data<Base> : public super_class { };

// # ---------------------------------------------------------------------------

#define BEFORE \
template<> \
struct Before<Base> : public virtual Data<Base> \
{ \
	Before( ); \
}; \
Before<Base>::Before(void)

// # ---------------------------------------------------------------------------

#define AFTER \
template<> \
struct After<Base> : public virtual Data<Base> \
{ \
	virtual ~After( ); \
}; \
After<Base>::~After(void)

// # ---------------------------------------------------------------------------

#define TEST(test_name) \
struct test_name : public Before<Base>, public After<Base>, public Base \
{ \
	void test( ); \
	static void wrapper( ); \
}; \
void test_name::wrapper(void) \
{ \
	test_name t; \
	try \
	{ \
		t.test(); \
	} \
	catch(std::string& s) \
	{ \
		throw s; \
	} \
	catch(std::exception& e) \
	{ \
		throw std::string(e.what()); \
	} \
	catch(const char *s) \
	{ \
		throw std::string(s); \
	} \
	catch(...) \
	{ \
		throw std::string("an error occured!"); \
	} \
} \
static ::lib::test::TestManager::Registrar register_##test_name(#test_name, &test_name::wrapper); \
void test_name::test(void)

// # ---------------------------------------------------------------------------

#endif

