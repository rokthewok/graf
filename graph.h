#ifndef JQRUFFER_GRAPH_H
#define JQRUFFER_GRAPH_H

#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <vector>
#include <utility>


/**
 * Graph:
 * features: BFS, DFS, weighted edges?, directed edges?,
 * direct access to all nodes
 * 
 * SHORTEST PATH:
 *      let queue
 *      let visited
 *      queue.enqueue(start_node)
 *      while queue not empty:
 *        node = queue.dequeue()
 *        if node is target_node:
 *          return path list
 *        for vertex connected to node:
 *          if vertex is unvisited:
 *            add to visited and mark parent as node
 *            queue.enqueue(vertex)
 */
namespace jqruffer {

template <typename Data, typename NodeKey = Data>
class Graph {
public:
  /**
   * \func    ctor
   */
  Graph();
  /**
   * \func    dtor
   */
  ~Graph();
  /**
   * \func    ctor
   * \brief   Copy ctor
   */
  Graph(const Graph & other);
  /**
   * \func    operator=
   * \brief   Copy assignment operator
   */
  Graph & operator=(const Graph & rhs);
  /**
   * \func        add
   * \brief       Add a node and its corresponding data to the graph.
   *              This will not attach the node to any other vertex; this
   *              is an isolated vertex.
   * \param key   The key of the node.
   * \param data  The data that corresponds to this node.
   */
  void add(const NodeKey & key, const Data & data);
  /**
   * \func        connect
   * \brief       Connect one vertex to another. This is a directed connection,
   *              so to make the connection bidirectional this function must be
   *              called a second time with the parameters reversed.
   * \param from  The node from which the edge originates.
   * \param to    The destination node.
   */
  void connect(const NodeKey & from, const NodeKey & to);
  /**
   * \func        get
   * \brief       Retrieve vertex data associated with the given key.
   * \param key   The key which corresponds to the vertex data.
   * \return      A modifiable reference to data.
   */
  Data & get(const NodeKey & key);
  /**
   * \func        get
   * \brief       Retrieve vertex data associated with the given key.
   * \param key   The key which corresponds to the vertex data.
   * \return      A non-modifiable reference to data.
   */
  const Data & get(const NodeKey & key) const;
  /**
   * \func        shortest_path
   * \brief       Find the shortest path between two nodes. Note that
   *              'shortest' is defined by the number of vertices between
   *              the start node and the end node. No weighting is performed
   *              on the edges.
   * \param from  The starting node.
   * \param to    The terminating node in the path search.
   * \param path  Out variable; will contain the shortest path between the
   *              two given vertices, or empty if no such path exists.
   */
  void shortest_path(const NodeKey & from,
                     const NodeKey & to,
                     std::vector<NodeKey> * path) const;
  /**
   * \func        size
   * \brief       Get the size (number of vertices) in the graph.
   * \return      The number of vertices in the graph.
   */
  size_t size() const;
private:
  typedef typename std::unordered_map<NodeKey, Data> Nodes;
  typedef typename Nodes::iterator NodesIt;
  typedef typename Nodes::const_iterator NodesCit;

  typedef typename std::vector<NodeKey> NodeKeys;
  typedef typename NodeKeys::iterator NodeKeysIt;
  typedef typename NodeKeys::const_iterator NodeKeysCit;

  typedef typename std::unordered_map<NodeKey, NodeKeys> AdjacencyList;
  typedef typename AdjacencyList::iterator AdjacencyListIt;
  typedef typename AdjacencyList::const_iterator AdjacencyListCit;

  typedef typename std::queue< std::pair<NodeKey, NodeKeys> > SearchPathQueue;

  typedef typename std::unordered_set<NodeKey> VisitedNodes;
  typedef typename VisitedNodes::iterator VisitedNodesIt;
  typedef typename VisitedNodes::const_iterator VisitedNodesCit;

  Nodes d_nodes;
  AdjacencyList d_graph;
};

template <typename Data, typename NodeKey>
Graph<Data, NodeKey>::Graph() 
  : d_nodes(),
    d_graph() {}

template <typename Data, typename NodeKey>
Graph<Data, NodeKey>::~Graph() {}

template <typename Data, typename NodeKey>
Graph<Data, NodeKey>::Graph(const Graph & other) 
  : d_nodes(other.d_nodes),
    d_graph(other.d_graph) {}

template <typename Data, typename NodeKey>
Graph<Data, NodeKey> & Graph<Data, NodeKey>::operator=(const Graph & rhs) {
  if(this != &rhs) {
    d_nodes = rhs.d_nodes;
    d_graph = rhs.d_graph;
  }

  return *this;
}

template <typename Data, typename NodeKey>
void Graph<Data, NodeKey>::connect(const NodeKey & from,
                                   const NodeKey & to) {
  AdjacencyListIt it = d_graph.find(from);
  if(d_graph.end() != it) {
    it->second.push_back(to);
  }
}

template <typename Data, typename NodeKey>
void Graph<Data, NodeKey>::add(const NodeKey & key,
                               const Data & data) {
  d_nodes.insert(std::make_pair(key, data));
  d_graph.insert(std::make_pair(key, NodeKeys()));
}

template <typename Data, typename NodeKey>
Data & Graph<Data, NodeKey>::get(const NodeKey & key) {
  return d_nodes.at(key);
}

template <typename Data, typename NodeKey>
const Data & Graph<Data, NodeKey>::get(const NodeKey & key) const {
  return d_nodes.at(key);
}

template <typename Data, typename NodeKey>
void Graph<Data, NodeKey>::shortest_path(const NodeKey & from,
                                         const NodeKey & to,
                                         std::vector<NodeKey> * path) const {
  SearchPathQueue search_queue;
  VisitedNodes visited;
  search_queue.push(std::make_pair(from, NodeKeys()));
  while(!search_queue.empty()) {
    std::pair<NodeKey, NodeKeys> node = search_queue.front();
    search_queue.pop();
    if(to == node.first) {
      *path = node.second;
      path->push_back(to);
      break;
    }

    const NodeKeys & edges = d_graph.at(node.first);
    for(auto vertex : edges) {
      if(visited.end() == visited.find(vertex)) {
        visited.insert(vertex);
        NodeKeys path = node.second;
        path.push_back(node.first);
        search_queue.push(std::make_pair(vertex, path));
      }
    }
  }
}

template <typename Data, typename NodeKey>
size_t Graph<Data, NodeKey>::size() const {
  return d_nodes.size();
}

}

#endif // JQRUFFER_GRAPH_H
