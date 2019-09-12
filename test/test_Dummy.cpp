#include <boost/test/unit_test.hpp>
#include <motors_weg_cvw300/Dummy.hpp>

using namespace motors_weg_cvw300;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    motors_weg_cvw300::DummyClass dummy;
    dummy.welcome();
}
