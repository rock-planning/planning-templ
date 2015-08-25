#include <boost/test/unit_test.hpp>
#include <templ/utils/CartographicMapping.hpp>

BOOST_AUTO_TEST_SUITE(cartographic_mapping)

BOOST_AUTO_TEST_CASE(longitude_latitude_to_metric)
{
    using namespace templ::utils;
    CartographicMapping mapping(CartographicMapping::MOON);
    base::Point point(16, 20.25, 0.0);
    base::Point metricPoint = mapping.latitudeLongitudeToMetric(point);

    BOOST_TEST_MESSAGE("Converted: longitude_latitude: " << point << " to metric: " << metricPoint);

    {
        base::Point location0(-8.00918, 16.00533, 0.0);
        base::Point location1(-8.00085, 17.00359, 0.0);

        base::Point location0metric = mapping.latitudeLongitudeToMetric(location0);
        base::Point location1metric = mapping.latitudeLongitudeToMetric(location1);

        double distance = (location0metric - location1metric).norm();

        BOOST_REQUIRE_MESSAGE(distance < 32E03 && distance > 30E03, "Distance around 30 km");
    }
}

BOOST_AUTO_TEST_SUITE_END()
