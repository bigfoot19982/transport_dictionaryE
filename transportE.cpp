#include <iostream>
#include <unordered_map>
#include <string_view>
#include <vector>
#include <variant>
#include <iomanip>
#include <sstream>
#include <optional>
#include <set>
#include <cmath>
#include "json.h"
#include "router.h"

using namespace std;

static int wait_time = 0;
static double velocity_of_buses = 0.0;

static const double p = 3.1415926535;

static bool FirstOutput = false;

long double toRadians(const long double degree) {
    long double one_deg = (p) / 180;
    return (one_deg * degree);
}
 
long double MyDistance(long double lat1, long double long1,
                     long double lat2, long double long2) {
    lat1 = toRadians(lat1);
    long1 = toRadians(long1);
    lat2 = toRadians(lat2);
    long2 = toRadians(long2);
     
    // Haversine Formula
    long double dlong = long2 - long1;
    long double dlat = lat2 - lat1;
 
    long double ans = pow(sin(dlat / 2), 2) +
                          cos(lat1) * cos(lat2) *
                          pow(sin(dlong / 2), 2);
 
    ans = 2 * asin(sqrt(ans));
    long double R = 6371000;
    ans = ans * R;
    return ans;
}

struct Coordinates {
    double latitude = 0.0;
    double longitude = 0.0;
};

struct BusInfo {
    size_t numberOfStops;
    size_t numberOfUniqueStops;
    double RoadLength;
    double curvature;
};

struct Wait {
    Wait(string StopName, double time) {
        StopName_ = StopName;
        time_ = time;
    }
    string StopName_;
    double time_ = 0.0;

    Json::Dict ParseTo() const {
      return Json::Dict{
          {"type", Json::Node("Wait"s)},
          {"stop_name", Json::Node(StopName_)},
          {"time", Json::Node(time_)},
      };
    }
};

struct Bus {
    Bus(string BusName, int span_count, double time) {
        BusName_ = BusName;
        span_count_ = span_count;
        time_ = time;
    }
    string BusName_;
    int span_count_ = 0;
    double time_ = 0.0;

    Json::Dict ParseTo() const {
      return Json::Dict{
          {"type", Json::Node("Bus"s)},
          {"bus", Json::Node(BusName_)},
          {"time", Json::Node(time_)},
          {"span_count", Json::Node(static_cast<int>(span_count_))}
      };
    }
};

struct ItineratyInfo {
    double total_time = 0.0;
    vector<variant<Bus, Wait>> items;
};

class Database {
public:
    void AddStop (string Stop, Coordinates coordinates, const unordered_map<string,int>& distances) {
        Stops[Stop] = coordinates;
        for (auto& [destination, dist] : distances) {
            if (distances_.find(destination) == distances_.end() || 
            distances_[destination].find(Stop) == distances_[destination].end()) {
                distances_[destination][Stop] = dist;
            }
            distances_[Stop][destination] = dist;
        }
    }

    void AddBus (string BusNumber, vector<string>& BusStops, bool Circled) {
        Buses[BusNumber] = {Circled, move(BusStops)};
        auto it = Buses.find(BusNumber);
        const vector<string>& basic_stops = (*it).second.second;
        for (const string_view& stop : basic_stops) {
            BusesThatMakeTheseStops[stop].insert((*it).first);
        }
    }

    variant<optional<set<string_view>>, bool> AskStop(const string& Stop) const {
        if (Stops.find(Stop) == Stops.end()) return false;
        if (BusesThatMakeTheseStops.find(Stop) == BusesThatMakeTheseStops.end()) return {};
        return BusesThatMakeTheseStops.at(Stop);
    }

    void ComputeBusData(const vector<string>& stopsByThisBus, double& dist_geo, int& dist_real, int& dist_return, size_t& unique) const {
        set<string> stops;
        for (int stop_now = 0; stop_now < stopsByThisBus.size(); ++stop_now) {
            stops.insert(stopsByThisBus[stop_now]);
            if (stop_now > 0) {
                dist_geo += MyDistance(Stops.at(stopsByThisBus[stop_now]).latitude, Stops.at(stopsByThisBus[stop_now]).longitude,
                                       Stops.at(stopsByThisBus[stop_now-1]).latitude, Stops.at(stopsByThisBus[stop_now-1]).longitude);

                dist_real += distances_.at(stopsByThisBus[stop_now-1]).at(stopsByThisBus[stop_now]);
                dist_return += distances_.at(stopsByThisBus[stop_now]).at(stopsByThisBus[stop_now-1]);
            }
        }
        unique = stops.size();
    }

    optional<BusInfo> AskBus(const string& BusNumber) const {
        if (Buses.find(BusNumber) == Buses.end()) {
            return nullopt;   
        } else {
            const vector<string>& stopsByThisBus = Buses.at(BusNumber).second;
            double dist_geo = 0.0;
            int dist_real = 0, dist_return = 0;
            size_t unique = 0;
            ComputeBusData (stopsByThisBus, dist_geo, dist_real, dist_return, unique);

            bool Circled = Buses.at(BusNumber).first;
            if (!Circled) {
                dist_geo *= 2;
                dist_real += dist_return;
            }
            return BusInfo{.numberOfStops = stopsByThisBus.size() * (Circled ? 1 : 2) - (Circled ? 0 : 1),
                           .numberOfUniqueStops = unique,
                           .RoadLength = static_cast<double>(dist_real),
                           .curvature = static_cast<double>(dist_real) / dist_geo};
        }
    }

    void ComputeAnsInfo(ItineratyInfo& ansInfo, int edge_count, int infoId,
    const Graph::DirectedWeightedGraph<double>& graph_, const Graph::Router<double>& router_) const {
        string prevBus = "";
        int cnt = 0, span = 0;
        double accumul_time = 0.0;
        for (int i = 0; i < edge_count; ++i) {
            const auto& nowEdge = graph_.GetEdge(router_.GetRouteEdge(infoId, i));
            if (i != 0 && (prevBus != nowEdge.Bus || nowEdge.is_end == true)) {
                ansInfo.items.push_back(Bus(prevBus, span, accumul_time - cnt * wait_time));
                span = 0, cnt = 0, accumul_time = 0.0;    
            }
            if (i == 0 || prevBus != nowEdge.Bus || nowEdge.is_end == true) 
                ansInfo.items.push_back(Wait(StopsFromVertexes.at(nowEdge.from), wait_time));
            
            span += nowEdge.span, cnt++, accumul_time += nowEdge.weight, prevBus = nowEdge.Bus;
        }
        ansInfo.items.push_back(Bus(prevBus, span, accumul_time - cnt * wait_time));
    }

    optional<ItineratyInfo> AskRoute(const string& from, const string& to, 
    const Graph::Router<double>& router_, const Graph::DirectedWeightedGraph<double>& graph_) const {
        if (from == to) return ItineratyInfo{0, {}};
        if (VertexesFromStop.find(from) == VertexesFromStop.end() 
        || VertexesFromStop.find(to) == VertexesFromStop.end()) return nullopt;

        optional<Graph::Router<double>::RouteInfo> info = router_.BuildRoute(VertexesFromStop.at(from), VertexesFromStop.at(to));
        if (info == nullopt) return nullopt;
        
        ItineratyInfo ansInfo;
        ansInfo.total_time = info->weight;
        ComputeAnsInfo(ansInfo, info->edge_count, info->id, graph_, router_);
        return ansInfo;
    }

    size_t GetVertexesNumber() const {return VertexId;}

    vector<Graph::Edge<double>> GetEdges() const {return edges;}

    void AddEdges() const {
        // вершин n, а вот ребер много будет для кажого автобуса и остановки будет свой
        for (const auto& [BusNumber, info] : Buses) {
            info.first ? AddEdgesCircled(info.second, BusNumber) : AddEdgesReturn(info.second, BusNumber);
        }
    }

private:
    unordered_map<string, Coordinates> Stops;
    unordered_map<string, unordered_map<string,int>> distances_;
    mutable unordered_map<string_view, set<string_view>> BusesThatMakeTheseStops;
    unordered_map<string, pair<bool, vector<string>>> Buses;

    mutable size_t VertexId = 0;
    mutable unordered_map<string, size_t> VertexesFromStop;
    mutable unordered_map<size_t, string> StopsFromVertexes;
    mutable vector<Graph::Edge<double>> edges;

    size_t GetVertexIndex(const string& stop) const {
        if (VertexesFromStop.find(stop) == VertexesFromStop.end()) {
            VertexesFromStop[stop] = VertexId;
            StopsFromVertexes[VertexId++] = stop;
        }
        return VertexesFromStop[stop];
    }

    double GetTime(const string& i, const string& j) const {
        if (distances_.find(i) != distances_.end() && distances_.at(i).find(j) != distances_.at(i).end()) {
            return (double(distances_.at(i).at(j)) / (velocity_of_buses*1000)) * 60; 
        } else if (distances_.find(j) != distances_.end() && distances_.at(j).find(i) != distances_.at(j).end()) {
            return (double(distances_.at(j).at(i)) / (velocity_of_buses*1000)) * 60;
        }
        return 0.0;
    }

    void AddEdge(const vector<string>& basic_stops, int i, int j, double time, const string& BusNumber, bool is_end, int span) const {
        size_t VertexIndexI = GetVertexIndex(basic_stops[i]);
        size_t VertexIndexJ = GetVertexIndex(basic_stops[j]);
        if (VertexIndexI != VertexIndexJ) {
            edges.push_back({VertexIndexI, VertexIndexJ, time, BusNumber, is_end, span});
        }
    }

    unordered_map<string_view, int> GetStopsOccurrences(const vector<string>& basic_stops) const {
        unordered_map<string_view, int> stops_occurrences;
        for (auto& str : basic_stops) stops_occurrences[str]++;
        return stops_occurrences;
    }

    void AddEdgesReturn(const vector<string>& basic_stops, const string& BusNumber) const {
        vector<double> there(basic_stops.size()-1, 0.0), here(basic_stops.size()-1, 0.0);
        unordered_map<string_view, int> stops_occurrences = GetStopsOccurrences(basic_stops);

        for (int stop_now = 1; stop_now < basic_stops.size(); ++stop_now) {
            there[stop_now-1] = GetTime(basic_stops[stop_now], basic_stops[stop_now-1]); // добираемся от i (где мы щас) до предыдущей остановки
            here[stop_now-1] = GetTime(basic_stops[stop_now-1], basic_stops[stop_now]);  // добираемся до сюда, до i от предыдущ.
            
            double here_collect = 0.0, there_collect = 0.0;
            for (int stops_behind = stop_now-1; stops_behind >= 0; --stops_behind) {    
                here_collect += here[stops_behind];
                there_collect += there[stops_behind];

                AddEdge(basic_stops, stop_now, stops_behind, there_collect + wait_time, BusNumber, 
                (stops_occurrences[basic_stops[stop_now]] > 1 ? true : false), stop_now - stops_behind);
                AddEdge(basic_stops, stops_behind, stop_now, here_collect + wait_time, BusNumber, 
                (stops_occurrences[basic_stops[stops_behind]] > 1 ? true : false), stop_now - stops_behind);
            }
        }
    }

    void AddEdgesCircled(const vector<string>& basic_stops, const string& BusNumber) const {
        unordered_map<string_view, int> stops_occurrences = GetStopsOccurrences(basic_stops);
        vector<double> here(basic_stops.size()-1, 0.0);
        double all_time = 0.0;

        for (int stop_now = 1; stop_now < basic_stops.size(); ++stop_now) {
            double time = GetTime(basic_stops[stop_now-1], basic_stops[stop_now]);
            here[stop_now-1] = time;
            all_time += time;

            AddEdge(basic_stops, stop_now-1, stop_now, time + wait_time, BusNumber, 
            (stops_occurrences[basic_stops[stop_now-1]] > 1 ? true : false), 1);
        }

        vector<double> pref(here.size()+1);
        for (int i = 1; i < pref.size(); ++i) {
            pref[i] = pref[i-1] + here[i-1];
        }

        for (int stop_now = 1; stop_now < basic_stops.size(); ++stop_now) {
            for (int stops_behind = stop_now-1; stops_behind >= 0; --stops_behind) {
                AddEdge(basic_stops, stop_now, 0, all_time - pref[stop_now] + wait_time, BusNumber, 
                (stops_occurrences[basic_stops[stop_now]] > 1 ? true : false), int(basic_stops.size() - stop_now - 1));
                
                AddEdge(basic_stops, 0, stops_behind, pref[stops_behind] + wait_time * 2, BusNumber, 
                true, stops_behind);
                
                AddEdge(basic_stops, stops_behind, stop_now, pref[stop_now] - pref[stops_behind] + wait_time, BusNumber, 
                (stops_occurrences[basic_stops[stops_behind]] > 1 ? true : false), stop_now - stops_behind);
            }
        }
    }
};

struct Request;
using RequestHolder = unique_ptr<Request>;

struct Request {
    enum class Type {
        ADD_STOP,
        ADD_BUS,
        ASK_BUS,
        ASK_STOP,
        ASK_ROUTE
    };
    Request(Type type) : type(type) {}
    static RequestHolder Create(Type type); 
    virtual void ParseFrom(const map<string, Json::Node>& input) = 0;
    virtual ~Request() = default;

    const Type type;
};

template <typename Info>
struct ReadRequest : Request {
    using Request::Request;
    virtual Json::Dict Process(const Database& database) const = 0;
};

template <typename Info>
struct ReadRequestRoute : Request {
    using Request::Request;
    virtual Json::Dict Process(const Database& database, const Graph::Router<double>& router_, 
    const Graph::DirectedWeightedGraph<double>& graph_) const = 0;
};

struct ModifyRequest : Request {
    using Request::Request;
    virtual void Process(Database& database) = 0;
};

struct AskRoute : ReadRequestRoute<ItineratyInfo> {
    AskRoute() : ReadRequestRoute(Type::ASK_ROUTE) {}
    void ParseFrom(const map<string, Json::Node>& input) {
        from = input.at(string("from")).AsString();
        to = input.at(string("to")).AsString();
        id_ = input.at("id").AsInt();
    }

    Json::Dict Process(const Database& database, const Graph::Router<double>& router_, 
    const Graph::DirectedWeightedGraph<double>& graph_) const {
        auto now = database.AskRoute(from, to, router_, graph_);

        Json::Dict dict;
        if (!now) {
            dict["error_message"] = Json::Node("not found"s);
        } else {
            dict["total_time"] = Json::Node(now->total_time);
            vector<Json::Node> items;
            items.reserve(now->items.size());
            for (const auto& item : now->items) {
                items.push_back(
                    visit([](const auto& type) -> Json::Dict {
                        return type.ParseTo();
                    }, 
                    item));
            }
            dict["items"] = move(items);
        }
        dict["request_id"] = id_;
        return dict;
    }

    string from;
    string to;
    int id_;
};

struct AskBus : ReadRequest<BusInfo> {
    AskBus() : ReadRequest(Type::ASK_BUS) {}
    void ParseFrom(const map<string, Json::Node>& input) {
        BusNumber = input.at(string("name")).AsString();
        id_ = input.at("id").AsInt();
    }

    Json::Dict Process(const Database& database) const {
        auto now = database.AskBus(BusNumber);

        Json::Dict dict;
        if (!now) {
            dict["error_message"] = Json::Node("not found"s);
        } else {
            dict = {
                {"stop_count", Json::Node(static_cast<int>(now->numberOfStops))},
                {"unique_stop_count", Json::Node(static_cast<int>(now->numberOfUniqueStops))},
                {"route_length", Json::Node(now->RoadLength)},
                {"curvature", Json::Node(now->curvature)},
            };
        }
        dict["request_id"] = id_;
        return dict;
    }

    string BusNumber;
    int id_;
};

struct AskStop : ReadRequest<set<string_view>> {
    AskStop() : ReadRequest(Type::ASK_STOP) {}
    void ParseFrom(const map<string, Json::Node>& input) {
        StopName = input.at(string("name")).AsString();
        id_ = input.at("id").AsInt();
    }

    Json::Dict Process(const Database& database) const {
        auto now = database.AskStop(StopName);
        Json::Dict dict;
        if (holds_alternative<bool>(now)) {
            dict["error_message"] = Json::Node("not found"s);
        } else {
            auto val = get<optional<set<string_view>>>(now);
            if (!val) {
                dict["buses"] = Json::Node({});    
            } else {
                vector<Json::Node> bus_nodes;
                for (const auto& bus_name : val.value()) {
                    bus_nodes.emplace_back(string(bus_name));
                }
                dict["buses"] = Json::Node(move(bus_nodes));
            }
        }
        dict["request_id"] = id_;
        return dict;
    }

    string StopName;
    int id_;
};

struct AddBus : ModifyRequest {
    AddBus() : ModifyRequest(Type::ADD_BUS) {}
    void ParseFrom(const map<string, Json::Node>& input) {
        Circled = input.at(string("is_roundtrip")).AsBool();
        BusNumber = input.at(string("name")).AsString();
        const auto& stops = input.at(string("stops")).AsArray();

        for (const auto& stop : stops) {
            BusStops.push_back(stop.AsString());
        }
    }

    void Process(Database& database) {
        database.AddBus(BusNumber, BusStops, Circled);
    }

    string BusNumber;
    vector<string> BusStops;
    bool Circled;
};

struct AddStop : ModifyRequest {
    AddStop() : ModifyRequest(Type::ADD_STOP) {}
    void ParseFrom(const map<string, Json::Node>& input) {
        StopName = input.at(string("name")).AsString();
        coordinates.latitude = input.at(string("latitude")).AsDouble();
        coordinates.longitude = input.at(string("longitude")).AsDouble();
        
        const auto& ma = input.at(string("road_distances")).AsMap();
        for (const auto& [k, v] : ma) {
            distances_[k] = v.AsInt();
        }
    }

    void Process(Database& database) {
        database.AddStop(StopName, coordinates, distances_);
    }

    string StopName;
    Coordinates coordinates;
    unordered_map<string, int> distances_;
};

RequestHolder Request::Create(Request::Type type) {
  switch (type) {
    case Request::Type::ADD_BUS:
      return make_unique<AddBus>();
    case Request::Type::ASK_BUS:
      return make_unique<AskBus>();
    case Request::Type::ADD_STOP:
      return make_unique<AddStop>();
    case Request::Type::ASK_STOP:
      return make_unique<AskStop>();
    case Request::Type::ASK_ROUTE:
      return make_unique<AskRoute>();
    default:
      return nullptr;
  }
}

const unordered_map<string_view, Request::Type> STR_TO_REQUEST_TYPE_FILL = {
    {"Stop", Request::Type::ADD_STOP},
    {"Bus", Request::Type::ADD_BUS}
};

const unordered_map<string_view, Request::Type> STR_TO_REQUEST_TYPE_ASK = {
    {"Stop", Request::Type::ASK_STOP},
    {"Bus", Request::Type::ASK_BUS},
    {"Route", Request::Type::ASK_ROUTE},
};

optional<Request::Type> ConvertRequestTypeFromString(string_view type_str, bool Fill) {
  if (const auto it = (Fill ? STR_TO_REQUEST_TYPE_FILL.find(type_str) : STR_TO_REQUEST_TYPE_ASK.find(type_str));
      it != (Fill ? STR_TO_REQUEST_TYPE_FILL.end() : STR_TO_REQUEST_TYPE_ASK.end())) {
    return it->second;
  }
  return nullopt;
}

RequestHolder ParseRequest(const map<string, Json::Node>& queries, bool Fill) {
  const auto request_type = ConvertRequestTypeFromString(queries.at(string("type")).AsString(), Fill);
  if (!request_type) {
    return nullptr;
  }
  RequestHolder request = Request::Create(*request_type);
  if (request) {
    request->ParseFrom(queries);
  };
  return request;
}

void SetWaitTimeAndVelocity(const Json::Document& doc) {
    wait_time = doc.GetRoot().AsMap().at(string("routing_settings")).AsMap().at("bus_wait_time").AsInt();
    velocity_of_buses = doc.GetRoot().AsMap().at(string("routing_settings")).AsMap().at("bus_velocity").AsDouble();
}

const vector<Json::Node>& GetArray(const Json::Document& doc, const string& query) {
    return doc.GetRoot().AsMap().at(query).AsArray();
}

vector<RequestHolder> ReadRequests(const vector<Json::Node>& vec, bool Fill) {
    vector<RequestHolder> requests;
    requests.reserve(vec.size());

    for (size_t i = 0; i < vec.size(); ++i) {
        if (auto request = ParseRequest(vec[i].AsMap(), Fill)) {
            requests.push_back(move(request));
        }
    }
    return requests;
}

void PrintAns(const Json::Document& root) {
    if (FirstOutput) {
        cout << ",";
    }
    Json::Print(root, cout);
    FirstOutput = true;
}

void ProcessRequestsFill(const vector<RequestHolder>& requests, Database& database) {
  for (const auto& request_holder : requests) {
    auto& request = static_cast<ModifyRequest&>(*request_holder);
    request.Process(database);   
  }
}

void ProcessRequestsAsk(const vector<RequestHolder>& requests, const Database& database, const Graph::Router<double>& router_, const Graph::DirectedWeightedGraph<double>& graph_) {
  using Json::Document;
  for (const auto& request_holder : requests) {
    if (request_holder->type == Request::Type::ASK_BUS) {
      const auto& request = static_cast<const AskBus&>(*request_holder);
      PrintAns(Document{request.Process(database)});
    } else if (request_holder->type == Request::Type::ASK_STOP) {
      const auto& request = static_cast<const AskStop&>(*request_holder);
      PrintAns(Document{request.Process(database)});
    } else if (request_holder->type == Request::Type::ASK_ROUTE) {
      const auto& request = static_cast<const AskRoute&>(*request_holder);
      PrintAns(Document{request.Process(database, router_, graph_)});
    }
  }
}

Graph::DirectedWeightedGraph<double> BuildGraph(const Database& database) {
    Graph::DirectedWeightedGraph<double> graph(database.GetVertexesNumber());
    for (const auto& edge : database.GetEdges()) {
        graph.AddEdge(edge);
    }
    return graph;
}

int main() {
    Database database;
    using namespace Json;

    const Document doc = Load(cin);
    
    SetWaitTimeAndVelocity(doc);
    const vector<Node>& fill = GetArray(doc, "base_requests");
    const vector<Node>& queries = GetArray(doc, "stat_requests");

    ProcessRequestsFill(ReadRequests(fill, true), database);

    database.AddEdges();
    Graph::DirectedWeightedGraph<double> graph_ = BuildGraph(database);
    Graph::Router<double> router_(graph_); 
    
    cout << "[";
    ProcessRequestsAsk(ReadRequests(queries, false), database, router_, graph_);
    cout << "\n]";
}