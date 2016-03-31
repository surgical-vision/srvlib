#include <srvlib/utils/config_reader.hpp>

srvlib::ConfigReader::ConfigReader(const std::string &config_file){

  std::ifstream ifs(config_file);
  if (!ifs.is_open()){
    throw std::runtime_error("Error, cannot open file!");
  }

  std::string line;
  while (std::getline(ifs, line))
  {
    if (line[0] == '#' || line.length() < 1) continue;
    remove_carriage_return(line);
    std::vector<std::string> tokens = split(line, '=');
    if (tokens.size() != 2) throw std::runtime_error("Error, bad parse!");
    config_[tokens[0]] = tokens[1];
  }
}


void srvlib::ConfigReader::remove_carriage_return(std::string& line) const {
  if (*line.rbegin() == '\r')
  {
    line.erase(line.length() - 1);
  }
}

bool srvlib::ConfigReader::has_element(const std::string &key) const {
  return config_.count(key) > 0;
}

std::string srvlib::ConfigReader::get_element(const std::string &key) const {

  if (!has_element(key)){
    throw std::runtime_error("Couldn't find key!");
  }
  else{
    std::map<std::string, std::string>::const_iterator it = config_.find(key);
    return it->second;
  }

}

std::vector<std::string> srvlib::ConfigReader::split(const std::string &s, char delim) const {
  std::vector<std::string> elems;
  split(s, delim, elems);
  return elems;
}

std::vector<std::string> &srvlib::ConfigReader::split(const std::string &s, char delim, std::vector<std::string> &elems) const {
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, delim)) {
    elems.push_back(item);
  }
  return elems;
}

