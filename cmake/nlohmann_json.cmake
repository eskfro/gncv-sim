# nlohmann/json as the target nlohmann_json::nlohmann_json (header only).
#
# Uses the system package when installed (sudo apt install nlohmann-json3-dev),
# otherwise downloads the release at configure time. To build offline without
# the package, use -DFETCHCONTENT_SOURCE_DIR_NLOHMANN_JSON=/path/to/json
include_guard(GLOBAL)

find_package(nlohmann_json 3.10 CONFIG QUIET)
if(NOT nlohmann_json_FOUND)
    include(FetchContent)
    FetchContent_Declare(nlohmann_json
        URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
        URL_HASH SHA256=d6c65aca6b1ed68e7a182f4757257b107ae403032760ed6ef121c9d55e81757d
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )
    FetchContent_MakeAvailable(nlohmann_json)
endif()
