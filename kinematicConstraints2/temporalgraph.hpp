#ifndef TEMPORALGRAPH_HPP
#define TEMPORALGRAPH_HPP

#include <utility>                          // for std::pair
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/bellman_ford_shortest_paths.hpp>
#include <boost/graph/graphviz.hpp>
//#include "event.hpp"
//#include "stconstraint.hpp"
#include <vector>

namespace STPSolver
{

struct Vertex
{
    std::string name;
    int x;
    int y;
};

struct Edge
{
    float weight;
//    float lowerBound;
//    float upperBound;
};

typedef boost::adjacency_list<
        boost::vecS, boost::vecS, boost::directedS,
        Vertex, Edge>
        Graph;


//typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, boost::property<boost::vertex_distance_t, int>, boost::property<boost::edge_weight_t, double>, int > Graph;
//typedef std::pair<int,int> Edge;
//typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;

typedef size_t EventId;

template <class T>
class Event
{
public:
    Event(EventId id)
        : id(id)
    {
    }

    EventId id;
    T time;
};

// inner class that is used to "pretty-print" to DOT file (VERTEX) ///////////////////////////////////
template <class GraphType>
class myVertexWriter {
public:
    explicit myVertexWriter(
            GraphType graph)
            : m_graph(graph)
    {

    }

    template <class Vertex> void operator()(std::ostream &out, const Vertex &v) const {
//        if (m_graph[v].x < 0) {
//            out << "[label=\"" << m_graph[v].name << "\"]";
//        } else {
            out << "[label=\"" << m_graph[v].name << "\" pos=\"" << m_graph[v].x << "," << m_graph[v].y << "!\"]";
//        }
    }
private:
    GraphType m_graph;
};  // end of inner class ///////////////////////////////////////////////////////////////////////

// inner class that is used to "pretty-print" to DOT file (EDGE) ///////////////////////////////////
template <class GraphType>
class myEdgeWriter {
public:
    explicit myEdgeWriter(GraphType _graphType) : graphType(_graphType) {}  // used to print a Graph_t to dot format
    template <class Edge> void operator()(std::ostream &out, const Edge &e) const {
//        auto u = source(e, graphType);
//        auto v = target(e, graphType);
//        auto weightMap = get(boost::edge_weight, graphType);

//        float weight = graphType[e].weight;
//        out << "[label=\"" << weight << "\"]";

//        out << "[src_agent_id=\"" << graphType[u].agent_id << "\", dst_agent_id=\"" << graphType[v].agent_id << "\"";
//        out << ", label=\"[";
//        if (graphType[e].ub == DBL_MAX) {
//            out << graphType[e].lb << ",&#8734;";
//        } else {
//            out << graphType[e].lb << "," << graphType[e].ub;
//        }
//        out << " t: " << graphType[e].translation << " r: " << graphType[e].rotation;
//        out << "]\"]";
    }
private:
    GraphType graphType;
};  // end of inner class ///////////////////////////////////////////////////////////////////////

template <class T>
class TemporalGraph
{
public:
    TemporalGraph();

    const EventId& startEvent() const {
        return m_startEvent;
    }

    const EventId& endEvent() const {
        return m_endEvent;
    }

    // adds an event and returns its unique identifier
    EventId addEvent(const std::string& name, int x, int y);

    // X_first - X_second must be in [lowerBound, upperBound]
    void addConstraint(
            const EventId& first,
            const EventId& second,
            T lowerBound,
            T upperBound);

    void solve();

    void getResult(
            const EventId& id,
            T& lowerBound,
            T& upperBound);

    void saveDotFile(
            const std::string& fileName)
    {
        // Generate DOT file for debug
        myVertexWriter<Graph> vw(m_G/*_reverse*/);  // instantiate the writer class
        myEdgeWriter<Graph> ew(m_G/*_reverse*/);  // instantiate the writer class
        std::ofstream dotFile(fileName);
        boost::write_graphviz(dotFile, m_G/*_reverse*/, vw, ew);
    }

private:
//    std::vector<Event<T> > EventList; // assumes that EventList[i].ID = i, EventList[0] is the reference event.
//    std::vector<STConstraint<T> > ConstraintList;
    std::vector<T> m_Event_lbs;
    std::vector<T> m_Event_ubs;
    Graph m_G;
//    Graph m_G_forward; // assumes that vertex ID of every event = Event.ID, vertex 0 is reference event.
//    Graph m_G_reverse; // assumes that vertex ID of every event = Event.ID, vertex 0 is reference event.

//    TemporalGraph();
//    void addEvent();
//    void addEvent(Event<T> &e);
//    void addConstraint(Event<T> &e1, Event<T> &e2, T c);
//    void tightenBounds();
    void tightenBounds(Graph &G, std::vector<T>& result, const EventId& start);
//    void printGraph(Graph &G);

    EventId m_startEvent;
    EventId m_endEvent;
};

template <class T>
TemporalGraph<T>::TemporalGraph()
{
    m_startEvent = addEvent("Start", -5, 5);
    m_endEvent = addEvent("End", 45, 5);
}

template <class T>
EventId TemporalGraph<T>::addEvent(
        const std::string& name, int x, int y
)
{
//    add_vertex(m_G_forward);
//    add_vertex(m_G_reverse);
//    return num_vertices(m_G_forward) - 1;
    auto v = add_vertex(m_G);
    m_G[v].name = name;
    m_G[v].x = x;
    m_G[v].y = y;
    return num_vertices(m_G) - 1;
}


template <class T>
void TemporalGraph<T>::addConstraint(
        const EventId& first,
        const EventId& second,
        T lowerBound,
        T upperBound)
{
    auto e = add_edge(first, second, m_G);
    m_G[e.first].weight = upperBound;

    e = add_edge(second, first, m_G);
    m_G[e.first].weight = -lowerBound;

//    m_G[e.first].upperBound = upperBound;
//    add_edge(first, second, upperBound, m_G);//m_G_forward);
//    add_edge(second, first, -lowerBound, m_G);//m_G_reverse);
}

template <class T>
void TemporalGraph<T>::solve()
{
    tightenBounds(m_G/*_forward*/, m_Event_ubs, m_startEvent);
    tightenBounds(m_G/*_reverse*/, m_Event_lbs, m_endEvent);
    T totalTime = -m_Event_lbs[m_startEvent];
    for(size_t i = 0; i < m_Event_lbs.size(); i++) {
        m_Event_lbs[i] = totalTime+m_Event_lbs[i];
    }
}

template <class T>
void TemporalGraph<T>::getResult(
        const EventId& id,
        T& lowerBound,
        T& upperBound)
{
    if (id < m_Event_lbs.size() && id < m_Event_ubs.size()) {
        lowerBound = m_Event_lbs[id];
        upperBound = m_Event_ubs[id];
    } else {
        lowerBound = std::numeric_limits<float>::infinity();
        upperBound = std::numeric_limits<float>::infinity();
    }
}

//template <class T>
//void TemporalGraph<T>::addEvent(Event<T> &e)
//{
//    e.ID = this->EventList.size();
//    this->EventList.push_back(e);
//    add_vertex(this->G_forward);
//    add_vertex(this->G_reverse);
//}
//
//template <class T> void TemporalGraph<T>::addConstraint(Event<T> &e1, Event<T> &e2, T c)
//{
//    this->ConstraintList.push_back(STConstraint<T>(e1, e2, c));
//    add_edge(e1.ID, e2.ID, c, this->G_forward);
//    add_edge(e2.ID, e1.ID, -c, this->G_reverse);
//}

template <class T>
void TemporalGraph<T>::tightenBounds(Graph &G, std::vector<T>& result, const EventId& start)
{
//    boost::property_map<Graph, boost::edge_weight_t>::type edge_weight = get(boost::edge_weight, G);

    boost::property_map<Graph, float Edge::*>::type edge_weight = get(&Edge::weight, G);

    result = std::vector<T>(num_vertices(G), std::numeric_limits<T>::infinity()); // Initialize all distances to infinity (max)
    result[start] = 0; // Make vertex 0 the source
    std::vector<size_t> parent(num_vertices(G));
    bool NoNegCycle = boost::bellman_ford_shortest_paths(G, num_vertices(G), edge_weight, &parent[0], &result[0], boost::closed_plus<T>(), std::less<T>(), boost::default_bellman_visitor());
    if(!NoNegCycle)
    {
        std::cerr << "Error: Negative Cycle in graph" << std::endl;
        result.clear();
    }
}

//template <class T> void TemporalGraph<T>::tightenBounds()
//{
//    this->Event_ubs = this->tightenBounds(G_forward);
//    this->Event_lbs = this->tightenBounds(G_reverse);
//    for(int i = 0; i < this->Event_lbs.size(); i++)
//        this->Event_lbs[i] = -this->Event_lbs[i];
//}
//
//template <class T> void TemporalGraph<T>::printGraph(Graph &G)
//{
//    std::cout << "Num Events = " << num_vertices(G) << std::endl;
//    std::cout << "Constraints:" << std::endl;
//    boost::graph_traits<Graph>::edge_iterator ei, ei_end;
//    for (boost::tie(ei, ei_end) = edges(G); ei != ei_end; ++ei)
//    {
//        boost::graph_traits<Graph>::edge_descriptor e = *ei;
//        Vertex u = boost::source(e, G), v = boost::target(e, G);
//        std::cout << "(v" << u << ",v" << v <<"): " << get(boost::edge_weight, G, e) << std::endl;
//    }
//}

}

#endif // TEMPORALGRAPH_H
