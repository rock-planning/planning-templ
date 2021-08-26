#include <boost/test/unit_test.hpp>
#include <templ/utils/CartographicMapping.hpp>
#include <templ/symbols/constants/Location.hpp>

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

        BOOST_REQUIRE_MESSAGE(distance < 32E03 && distance > 30E03, "Distance around 30 km, but was " << distance);
    }

    // The mapping is not suited for computation of distance close to poles
    // thus refer to law of cosine based computation in Location
    //
    {
        base::Point location0(-83.34083, 84.64467, 0.0);
        base::Point location1(-83.54570, 87.09851, 0.0);

        base::Point location0metric = mapping.latitudeLongitudeToMetric(location0);
        base::Point location1metric = mapping.latitudeLongitudeToMetric(location1);
        double distance = (location0metric - location1metric).norm();

        //BOOST_REQUIRE_MESSAGE(distance < 10.75E03 && distance > 10.5E03,
        BOOST_TEST_MESSAGE(
                "Distance around 10.5 km, but was " << distance);


        using namespace templ::symbols::constants;
        Location a("a", location0, CartographicMapping::RADIUS_MOON_IN_M);
        Location b("b", location1, CartographicMapping::RADIUS_MOON_IN_M);

        double s_distance = Location::getDistance(a,b);

        BOOST_REQUIRE_MESSAGE(s_distance < 10.75E03 && s_distance > 10.5E03,
                "Distance around 10.5 km, but was " << s_distance);
    }
}

BOOST_AUTO_TEST_SUITE_END()
