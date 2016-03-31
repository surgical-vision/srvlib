#pragma once

/**

srvlib - A robotics visualizer specialized for the da Vinci robotic system.
Copyright (C) 2014 Max Allan

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

**/

#include <string>
#include <fstream>
#include <map>
#include <vector>
#include <sstream>

namespace srvlib {

  /**
  * @class ConfigReader
  * @brief A simple configuration file reader.
  * Very basic functionality for reading parameters from a configuration file. Does nothing fancy with it.
  * Assumes that options are specified in the format key=value with each key-value pair on a new line. Comments are specified by lines starting with a \#.
  */
  class ConfigReader {

  public:

    /**
    * Open a ConfigReader from a file.
    * @param The full path to the configuration file.
    */
    explicit ConfigReader(const std::string &config_file);

    /**
    * Get rid of the windows carriage return on lines.
    * @param[in,out] A line that may or may not have any carriage returns in it. After this call it will have precisely none.
    */
    void remove_carriage_return(std::string& line) const;

    /**
    * Check if we have an element with a particular key.
    * @param[in] key The key to search for.
    */
    bool has_element(const std::string &key) const;

    /**
    * Return the value associated wtih the key. Raises an exception if not found.
    * @param[in] key The key to search for.
    * @return The value associated with key.
    */
    std::string get_element(const std::string &key) const;

    /**
    * Generic method to get elements and cast them to fundamental type
    * @param[in] key The key to search for.
    * @return The value associated with key.
    */
    template<typename T>
    T get_element_as_type(const std::string &key) const;

  protected:

    /**
    * Helper method to split a string on a token.
    * @param[in] s The string to split.
    * @param[in] delim The char to use as a token.
    * @return A vector of components.
    */
    std::vector<std::string> split(const std::string &s, char delim) const;

    /**
    * Helper method to split a string on a token.
    * @param[in] s The string to split.
    * @param[in] delim The char to use as a token.
    * @param[out] elems The elements of the split string.
    * @return A vector of components.
    */
    std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) const;


    std::map<std::string, std::string> config_; /**< Simple map containing the configuration options. */


  };

  /**
  * Generic method to get elements and cast them to fundamental type
  * @param[in] key The key to search for.
  * @return The value associated with key.
  */
  template<typename T>
  T ConfigReader::get_element_as_type(const std::string &key) const {

    if (config_.count(key) == 0){
      throw std::runtime_error("Couldn't find key!");
    }
    else{
      std::map<std::string, std::string>::const_iterator it = config_.find(key);
      std::stringstream s(it->second);
      T x;
      s >> x;
      return x;
    }

  }



}