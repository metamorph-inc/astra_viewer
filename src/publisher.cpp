#include <string>
#include <map>

#include "publisher.h"

namespace astra_viewer {

    const std::map<std::string, Publisher::StreamType> Publisher::streamType_enumMapper({
        {"depth", Publisher::StreamType::DEPTH},
        {"color", Publisher::StreamType::COLOR},
        {"ir16", Publisher::StreamType::IR16},
        {"irrgb", Publisher::StreamType::IRRGB}
    });

}