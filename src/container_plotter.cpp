#include <log2plot/plot/container_plotter.h>
#include <iostream>
namespace log2plot
{

ContainerPlotter::ContainerPlotter(LogType _type, std::string _legend) :
  GenericContainer(_type), legend(decompose(_legend))
{ }

// write additionnal info
void ContainerPlotter::writeInfo(const std::string &_label, const std::string &_info)
{
  GenericContainer::writeInfo(_label, _info);

  // hijack line type
  if(_label == "lineType")
    line_type = decompose(_info);
}

std::vector<std::string> ContainerPlotter::decompose(std::string list)
{
  std::vector<std::string> tokens;
  if(list.size())
  {
    if(list.substr(0, 1) == "[")  // has brackets
      list = list.substr(1, list.size()-2);

    std::string token;
    std::istringstream tokenStream(list);
    while (std::getline(tokenStream, token, ','))
    {
      size_t start(0), end(token.size()-1);

      while(token[start] == ' ' || token[start] == '\'')
        start++;
      while(token[end] == ' ' || token[end] == '\'')
        end--;
      tokens.push_back(token.substr(start, end+1-start));
    }
  }
  return tokens;
}

std::vector<std::pair<size_t, size_t>> ContainerPlotter::decomposeGraph(const std::string & graph_s)
{
  std::vector<std::pair<size_t, size_t>> graph;
  bool node1_ok(false);

  std::string node1, node2;

  for(const unsigned char &c: graph_s)
  {
    if(std::isdigit(c))
    {
      if(node1_ok) node2 += c;
      else        node1 += c;
    }
    else if(node2.length())
    {
        graph.push_back({std::stoi(node1), std::stoi(node2)});
        node1.clear();
        node2.clear();
        node1_ok = false;
    }
    else if(node1.length())
      node1_ok = true;
  }
  return graph;
}

std::string ContainerPlotter::format(size_t idx) const
{
  if(idx < line_type.size())
    return line_type[idx];
  return "";
}
std::string ContainerPlotter::label(size_t idx) const
{
  if(idx < legend.size())
    return legend[idx];
  return "";
}

}

