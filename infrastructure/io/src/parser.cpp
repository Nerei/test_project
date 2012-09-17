#include <pcl2/io/parser.hpp>

namespace pcl {
  namespace {
    std::vector<Parser *> all_parsers;
  }

  const std::vector<Parser *> & Parser::getAll() {
    return all_parsers;
  }
}
