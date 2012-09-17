#pragma once

#include <istream>
#include <vector>

#include <pcl2/io/source.hpp>

namespace pcl {

  struct Parser {

    virtual ~Parser() {}

    virtual Source * parseStream(std::istream & stream) = 0;
    virtual std::string getPreferredFileExtension() { return ""; }

    static const std::vector<Parser *> & getAll();

  private:
    Parser(const Parser &);
    Parser & operator = (const Parser &);
  };

}
