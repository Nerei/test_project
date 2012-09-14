#pragma once

#include <string>
#include <vector>

#include <pcl2/io/source.hpp>

namespace pcl {

  struct FileSourceFactory {
    virtual Source * openFile(const std::string & file_name) = 0;
    virtual std::string getPreferredFileExtension() { return ""; }

    static const std::vector<FileSourceFactory *> & getAll();

  private:
    FileSourceFactory(const FileSourceFactory &);
    FileSourceFactory & operator = (const FileSourceFactory &);
  };

}
