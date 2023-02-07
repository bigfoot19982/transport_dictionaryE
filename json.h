#pragma once

#include <istream>
#include <map>
#include <string>
#include <variant>
#include <vector>

namespace Json {
  class Node;
  using Dict = std::map<std::string, Node>;

  class Node : std::variant<std::vector<Node>,
                            std::map<std::string, Node>,
                            int,
                            std::string,
                            bool,
                            double> {
  public:
    using variant::variant;
    const variant& GetBase() const { return *this; }

    const auto& AsArray() const {
      return std::get<std::vector<Node>>(*this);
    }
    const auto& AsMap() const {
      return std::get<std::map<std::string, Node>>(*this);
    }
    int AsInt() const {
      return std::get<int>(*this);
    }
    const auto& AsString() const {
      return std::get<std::string>(*this);
    }
    const auto& AsBool() const {
      return std::get<bool>(*this);
    }
    const auto& AsDouble() const {
      return std::get<double>(*this);
    }
  };

  class Document {
  public:
    explicit Document(Node root);

    const Node& GetRoot() const;

  private:
    Node root;
  };

  Document Load(std::istream& input);

  void PrintNode(const Node& node, std::ostream& output);

  template <typename Value>
  void PrintValue(const Value& value, std::ostream& output);

  template <>
  void PrintValue<std::string>(const std::string& value, std::ostream& output);

  template <>
  void PrintValue<bool>(const bool& value, std::ostream& output);

  template <>
  void PrintValue<std::vector<Node>>(const std::vector<Node>& nodes, std::ostream& output);

  template <>
  void PrintValue<Dict>(const Dict& dict, std::ostream& output);

  void Print(const Document& document, std::ostream& output);
}
