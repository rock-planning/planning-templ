#include <boost/test/unit_test.hpp>

#include <templ/utils/CSVLogger.hpp>
#include <sstream>

using namespace templ;

BOOST_AUTO_TEST_SUITE(utils)

BOOST_AUTO_TEST_CASE(csv_logger)
{

    CSVLogger logger({"col-0","col-1","col-2"});

    CSVLogger::ColumnDescription description;
    for(size_t i = 0; i < 10; ++i)
    {
        std::stringstream ss;
        ss << "col-" << i;
        description.push_back(ss.str());
    }
    logger.setColumnDescription(description);

    logger.prepareRow();
    for(size_t i = 0; i < 10; ++i)
    {
        logger.appendToRow(i*1.0);
    }
    logger.commitRow();

    for(size_t i = 0; i < 10; ++i)
    {
        double value = logger.get(0,i);
        BOOST_REQUIRE_MESSAGE(value == i*1.0, "Column with value " << i*1.0 << " expected: " << " was " << value);

        std::stringstream ss;
        ss << "col-" << i;
        BOOST_REQUIRE_MESSAGE(ss.str() == logger.getColumnDescription()[i], "Column description is '" << ss.str() << "'");
    }

    logger.save("/tmp/templ-test-utils-csvlogger.csv");

}

BOOST_AUTO_TEST_SUITE_END()
