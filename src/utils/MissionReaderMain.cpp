#include <templ/io/MissionReader.hpp>

int main(int argc, char** argv)
{
    if(argc == 1)
    {
        printf("usage: %s <file> \n", argv[0]);
        exit(-1);
    }

    std::string filename = argv[1];
    using namespace templ;
    Mission mission = io::MissionReader::fromFile(filename);
    printf("%s\n",mission.toString().c_str());

    return 0;
}
