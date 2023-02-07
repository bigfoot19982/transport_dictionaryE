#include "json.h"
#include <sstream>
#include <algorithm>
#include <iostream>
#include <iomanip>

using namespace std;

namespace Json {

  Document::Document(Node root) : root(move(root)) {
  }

  const Node& Document::GetRoot() const {
    return root;
  }

  Node LoadNode(istream& input);

  Node LoadArray(istream& input) {
    vector<Node> result;

    for (char c; input >> c && c != ']'; ) {
      if (c != ',') {
        input.putback(c);
      }
      result.push_back(LoadNode(input));
    }

    return Node(move(result));
  }

  Node LoadInt(istream& input) {
    int result = 0;
    while (isdigit(input.peek())) {
      result *= 10;
      result += input.get() - '0';
    }
    return Node(result);
  }

  Node LoadString(istream& input) {
    string line;
    getline(input, line, '"');
    return Node(move(line));
  }

  Node LoadBool(istream& input) {
    input >> ws;
    bool b = false;
    string now;
    while (isalpha(input.peek())) {
      now += input.get();
    }
    if (now == "true") b = true;
    return Node(b);
  }

  Node LoadDouble(istream& input) {
    input >> ws;
    double d;
    input >> d;
    return Node(d);
  }

  Node LoadDict(istream& input) {
    map<string, Node> result;

    for (char c; input >> c && c != '}'; ) {
      if (c == ',') {
        input >> c;
      }

      string key = LoadString(input).AsString();
      input >> c;

      if (key == "latitude" || key == "longitude" || key == "bus_velocity") { 
        result.emplace(move(key), LoadDouble(input));
      } else if (key == "is_roundtrip") {
        result.emplace(move(key), LoadBool(input));
      } else {
        result.emplace(move(key), LoadNode(input));
      }
    }

    return Node(move(result));
  }

  Node LoadNode(istream& input) {
    char c;
    input >> c;

    if (c == '[') {
      return LoadArray(input);
    } else if (c == '{') {
      return LoadDict(input);
    } else if (c == '"') {
      return LoadString(input);
    } else {
      input.putback(c);
      return LoadInt(input);
    }
  }

  Document Load(istream& input) {
    return Document{LoadNode(input)};
  }

  template <typename T>
  void PrintValue(const T& obj, ostream& output) {
    output << obj;
  }

  template <>
  void PrintValue<string>(const string& value, ostream& output) {
    output << '"' << value << '"';
  }

  template <>
  void PrintValue<bool>(const bool& value, std::ostream& output) {
    output << std::boolalpha << value;
  }

  template <>
  void PrintValue<std::vector<Node>>(const std::vector<Node>& nodes, std::ostream& output) {
    output << '[';
    bool first = true;
    for (const Node& node : nodes) {
      if (!first) {
        output << ", ";
      }
      first = false;
      PrintNode(node, output);
    }
    output << ']';
  }

  template <>
  void PrintValue<Dict>(const Dict& dict, std::ostream& output) {
    output << '{';
    bool first = true;
    for (const auto& [key, node]: dict) {
      if (!first) {
        output << ", ";
      }
      first = false;
      PrintValue(key, output);
      output << ": ";
      PrintNode(node, output);
    }
    output << '}';
  }

  void PrintNode(const Json::Node& node, ostream& output) {
    visit([&output](const auto& value) { PrintValue(value, output); },
          node.GetBase());
  }

  void Print(const Document& document, ostream& output) {
    PrintNode(document.GetRoot(), output);
  }
}
