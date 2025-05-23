#include <openpose/filestream/peopleJsonSaver.hpp>
#include <openpose/filestream/fileStream.hpp>
#include <iostream>

namespace op
{
    PeopleJsonSaver::PeopleJsonSaver(const std::string& directoryPath) :
        FileSaver{directoryPath}
    {
    }

    PeopleJsonSaver::~PeopleJsonSaver()
    {
    }

    void PeopleJsonSaver::save(
        const std::vector<std::pair<Array<float>, std::string>>& keypointVector,
        const std::vector<std::vector<std::array<float,3>>>& candidates, const std::string& fileName,
        const bool humanReadable) const
    {
        try
        {
            // Record json
            const auto finalFileName = getNextFileName(fileName) + ".json";
            // savePeopleJson(keypointVector, candidates, finalFileName, humanReadable);
            savePeopleTxt(keypointVector);
        }
        catch (const std::exception& e)
        {
            error(e.what(), __LINE__, __FUNCTION__, __FILE__);
        }
    }
}
