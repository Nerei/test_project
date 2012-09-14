#include <pcl2/io/file_source_factory.hpp>

namespace pcl {
  namespace {
    std::vector<FileSourceFactory *> all_file_source_factories;
  }

  const std::vector<FileSourceFactory *> & FileSourceFactory::getAll() {
    return all_file_source_factories;
  }
}
