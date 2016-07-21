// Copyright @ 2016 Caoyang Jiang

/**
 * section: xmlReader
 * synopsis: Parse an XML file with an xmlReader
 * purpose: Demonstrate the use of xmlReaderForFile() to parse an XML file
 *          and dump the informations about the nodes found in the process.
 *          (Note that the XMLReader functions require libxml2 version later
 *          than 2.6.)
 * usage: reader1 <filename>
 * test: reader1 test2.xml > reader1.tmp && diff reader1.tmp
 * $(srcdir)/reader1.res
 * author: Daniel Veillard
 * copy: see Copyright for the status of this software.
 */

#include <libxml/xmlreader.h>
#include <stdio.h>

#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

class MPD
{
 public:
  int i = 0;
};

class Node
{
 public:
  Node()
  {
  }
  ~Node()
  {
  }

  void Pushback(const Node &subnode)
  {
    subnodes_.push_back(subnode);
  }

  void SetTag(const std::string &tag)
  {
    tag_ = tag;
  }

  void SetValue(const std::string &value)
  {
    value_ = value;
  }

  void AddAttr(const std::string &attrname, const std::string &attrproperty)
  {
    attributes_[attrname] = attrproperty;
  }

  const std::string &GetTag() const
  {
    return tag_;
  }

  const std::string &GetValue() const
  {
    return value_;
  }

  const std::string &LookUpAttrPropertyByName(const std::string &attrname)
  {
    return attributes_[attrname];
  }

  const std::map<const std::string, std::string> &GetAttrs() const
  {
    return attributes_;
  }

  size_t GetNumOfAttr() const
  {
    return attributes_.size();
  }

  size_t GetNumOfSubNodes() const
  {
    return subnodes_.size();
  }

  const Node &GetSubNodeByIndex(size_t subnodeidx) const
  {
    return subnodes_[subnodeidx];
  }

 private:
  // Subnodes
  std::vector<Node> subnodes_;

  // Node information
  std::string tag_;
  std::string value_;
  std::map<const std::string, std::string> attributes_;
};

/**
 * @brief      Parse XML file into tree-structured nodes.
 */
class XMLParser
{
 public:
  XMLParser()
  {
    root_ = std::make_shared<Node>();
  }
  ~XMLParser()
  {
  }

  bool Parse(const std::string filename)
  {
    reader_ = xmlReaderForFile(filename.c_str(), NULL, 0);

    if (reader_ == NULL) return false;

    // Get things going
    int ret      = xmlTextReaderRead(reader_);
    int nodetype = xmlTextReaderNodeType(reader_);

    while ((nodetype == NodeType::COMMENT) ||
           (nodetype == NodeType::WHITESPACE))
    {
      ret      = xmlTextReaderRead(reader_);
      nodetype = xmlTextReaderNodeType(reader_);
    }

    if (nodetype == NodeType::ELEMENT)
    {
      root_->SetTag(std::string(
          reinterpret_cast<const char *>(xmlTextReaderConstName(reader_))));

      if (xmlTextReaderHasAttributes(reader_))
      {
        while (xmlTextReaderMoveToNextAttribute(reader_))
        {
          root_->AddAttr(std::string(reinterpret_cast<const char *>(
                             xmlTextReaderConstName(reader_))),
                         std::string(reinterpret_cast<const char *>(
                             xmlTextReaderConstValue(reader_))));
        }
      }
    }
    else
    {
      std::cout << "[ERROR]: First node is not an element node." << std::endl;
    }

    // Recursively parsing nodes (Depth first)
    Recurse(*root_);

    xmlFreeTextReader(reader_);

    RecursePrint(*root_);
    return true;
  }

  void RecursePrint(const Node &node) const
  {
    std::cout << node.GetTag() << ": " << node.GetValue() << std::endl;
    std::cout << "|-" << std::endl;

    for (std::map<const std::string, std::string>::const_iterator it =
             node.GetAttrs().begin();
         it != node.GetAttrs().end();
         it++)
    {
      std::cout << "  AttrName: " << it->first
                << " , AttrProperty: " << it->second << std::endl;
    }

    for (size_t sz = 0; sz < node.GetNumOfSubNodes(); sz++)
    {
      RecursePrint(node.GetSubNodeByIndex(sz));
    }
  }

  std::shared_ptr<Node> &GetRoot()
  {
    return root_;
  }

 private:
  int Recurse(Node &parenode)
  {
    int ret      = xmlTextReaderRead(reader_);
    int nodetype = xmlTextReaderNodeType(reader_);

    while ((nodetype == NodeType::COMMENT) ||
           (nodetype == NodeType::WHITESPACE))
    {
      ret      = xmlTextReaderRead(reader_);
      nodetype = xmlTextReaderNodeType(reader_);
    }

    if (nodetype == NodeType::ELEMENT)
    {
      // This implement may not work well for very large XML file.
      Node node;
      node.SetTag(std::string(
          reinterpret_cast<const char *>(xmlTextReaderConstName(reader_))));

      if (xmlTextReaderHasAttributes(reader_))
      {
        while (xmlTextReaderMoveToNextAttribute(reader_))
        {
          node.AddAttr(std::string(reinterpret_cast<const char *>(
                           xmlTextReaderConstName(reader_))),
                       std::string(reinterpret_cast<const char *>(
                           xmlTextReaderConstValue(reader_))));
        }
      }

      // Process all element typed subnodes in the next depth level.
      while (Recurse(node) == 1)
      {
      };

      parenode.Pushback(node);

      return ret;
    }
    else if (nodetype == NodeType::TEXT)
    {
      xmlChar *str = xmlTextReaderReadString(reader_);
      parenode.SetValue(std::string(reinterpret_cast<const char *>(str)));
      xmlFree(str);
      Recurse(parenode);
      return 0;  // Stop parsing further
    }
    else if ((nodetype == END) || (ret == 0))
    {
      return 0;  // Reach end
    }
    else if (nodetype == -1)
    {
      std::cout << "[ERROR]: error nodetype" << std::endl;
    }
    else
    {
      std::cout << "[ERROR]: NodeType " << nodetype << " not handled"
                << std::endl;
    }
  }

 private:
  enum NodeType
  {
    ELEMENT    = 1,
    TEXT       = 3,
    COMMENT    = 8,
    WHITESPACE = 14,
    END        = 15,
  };

  xmlTextReaderPtr reader_;
  std::shared_ptr<Node> root_;
};

/**
 * @brief      Parse MPD XML file into data structure.(Using decorator pattern.)
 */
class MpdParser
{
 public:
  MpdParser()
  {
  }

  ~MpdParser()
  {
  }

  bool Parse(const std::string filename)
  {
    if (!xmlparser_.Parse(filename)) return false;

    if (!NodeToMpd(xmlparser_.GetRoot())) return false;
  }

  std::shared_ptr<MPD> &GetMpd()
  {
    return mpd_;
  }

 private:
  bool NodeToMpd(std::shared_ptr<Node> &root)
  {
  }

 private:
  XMLParser xmlparser_;
  std::shared_ptr<MPD> mpd_;
};

int main(int argc, char **argv)
{
  XMLParser xmlparser;
  xmlparser.Parse(std::string(argv[1]));
  return 0;
}

// static void processNode(xmlTextReaderPtr reader)
// {
//   const xmlChar *name, *value;

//   name                   = xmlTextReaderConstName(reader);
//   if (name == NULL) name = BAD_CAST "--";

//   value = xmlTextReaderConstValue(reader);

//   printf("%d %d %s %d %d",
//          xmlTextReaderDepth(reader),
//          xmlTextReaderNodeType(reader),
//          name,
//          xmlTextReaderIsEmptyElement(reader),
//          xmlTextReaderHasValue(reader));

//   printf("Attr count: %d\n", xmlTextReaderAttributeCount(reader));
//   if (name)
//     if (value == NULL)
//       printf("\n");
//     else
//     {
//       if (xmlStrlen(value) > 40)
//         printf(" %.40s...\n", value);
//       else
//         printf(" %s\n", value);
//     }
// }

// static void streamFile(const char *filename)
// {
//   xmlTextReaderPtr reader;
//   Node root;
//   int ret;

//   reader = xmlReaderForFile(filename, NULL, 0);
//   if (reader != NULL)
//   {
//     ret = xmlTextReaderRead(reader);
//     while (ret == 1)
//     {
//       processNode(reader);
//       ret = xmlTextReaderRead(reader);
//     }
//     xmlFreeTextReader(reader);
//     if (ret != 0)
//     {
//       fprintf(stderr, "%s : failed to parse\n", filename);
//     }
//   }
//   else
//   {
//     fprintf(stderr, "Unable to open %s\n", filename);
//   }
// }

// int main(int argc, char **argv)
// {
//   if (argc != 2) return (1);

//   LIBXML_TEST_VERSION

//   streamFile(argv[1]);

//   xmlCleanupParser();

//   xmlMemoryDump();
//   return (0);
// }

// #else
// int main(void)
// {
//   fprintf(stderr, "XInclude support not compiled in\n");
//   exit(1);
// }
// #endif
