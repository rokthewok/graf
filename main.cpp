#include <iostream>
#include <vector>
#include <string>

#include "graph.h"

#define TEST_TRUE(a) if(a) std::cout << #a << " IS TRUE." << std::endl; \
                     else std::cout << #a << " IS FALSE." << std::endl;

template <typename T, typename D>
void print_path(std::ostream & out,
                const typename std::vector<T> & path,
                const typename jqruffer::Graph<T, D> & graph) {
  for(auto node : path) {
    out << graph.get(node) << " ";
  }

  out << std::endl;
}

int main(int argc, char ** argv) {
  jqruffer::Graph<std::string> graph;
  graph.add("A", "Aardvark");
  graph.add("B", "Badger");
  graph.add("C", "Cat");
  graph.add("D", "Dog");
  graph.add("E", "Elephant");
  graph.add("F", "Fox");

  graph.connect("A", "B");
  graph.connect("B", "C");
  graph.connect("C", "D");
  graph.connect("B", "E");
  graph.connect("B", "F");
  graph.connect("E", "F");

  std::vector<std::string> path;

  graph.shortest_path("A", "F", &path);
  TEST_TRUE(3 == path.size());
  TEST_TRUE("A" == path[0]);
  TEST_TRUE("B" == path[1]);
  TEST_TRUE("F" == path[2]);
  print_path(std::cout, path, graph);

  path.clear();
  graph.shortest_path("A", "B", &path);
  print_path(std::cout, path, graph);

  jqruffer::Graph<std::string> new_graph(graph);
  TEST_TRUE(6 == new_graph.size());

  return 0;
}
