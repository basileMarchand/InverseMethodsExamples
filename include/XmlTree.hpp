#ifndef _XML_TREE_H_
#define _XML_TREE_H_


#include "ext_tinyxml2.hpp"

using tinyxml2::XMLDocument;
using tinyxml2::XMLNode;
using tinyxml2::XMLElement;
using tinyxml2::XMLAttribute;
using tinyxml2::XMLError;
using tinyxml2::XML_SUCCESS;

#include "StringUtilities.hpp"

#define CHECK_XML( code ) \
  if ( code != XML_SUCCESS) { \
    std::cerr << "Error in the XML file " << std::endl; }


class XmlBlock{
public:
  XmlBlock();
  XmlBlock( XMLElement* );
  XmlBlock( XmlBlock& );

  virtual void fromFile(const string& fpath);
  vector<string> listRoot();
  vector<string> listBlocks();
  XmlBlock* getBlock( string, int offset=0 );

  bool elemExists(const string& );

  string getStrAttr(const string& name, bool parent=false){  
    const char * val;
    if(parent){
      val = this->root->Attribute( name.c_str() );
    }
    else{
      val = this->current->Attribute( name.c_str() );
    }
    string ret(val);
    return ret;
  }

  string elemId(){
    string ret( this->current->Name() );
    return ret;
  }

  string getStrElem(){
    const char* val;
    val = this->current->GetText();
    string res(val);
    return res;
  }

  string getStrElem(const string& name){  
    const char * val;
    XMLElement* tmp = this->current;
    while( string(tmp->Name()) != name ){
      tmp = tmp->NextSiblingElement();
    }
    val = tmp->GetText( );
    string ret(val);
    //_erase_space( ret );   
    return ret;
  }

  vector<int> getVectorIntAttr(const string& name){
    string val = this->getStrAttr( name );
    vector<int> ret = STRUTIL::Tokenize<int>(val, ",");
    return ret;
  }

  vector<int> getVectorIntElem(const string& name){
    string val = this->getStrElem( name );
    vector<int> ret = STRUTIL::Tokenize<int>(val, ",");
    return ret;
  }

  vector<double> getVectorDoubleElem(const string& name){
    string val = this->getStrElem( name );
    vector<double> ret = STRUTIL::Tokenize<double>(val, ",");
    return ret;
  }

  vector<string> getVectorStrElem(const string& name){
    string val = this->getStrElem( name );
    vector<string> ret = STRUTIL::Tokenize<string>(val, ",");
    return ret;
  }

  vector<double> getVectorDoubleElem(){
    string val = this->getStrElem();
    vector<double> ret = STRUTIL::Tokenize<double>(val, ",");
    return ret;
  }

  double getDoubleAttr(const string& name, bool parent=false){  
    double val;
    if(parent){
      //XMLAttribute* at = this->root->FindAttribute( name.c_str() );
      this->root->QueryDoubleAttribute( name.c_str(), &val );
    }
    else{
      this->current->QueryDoubleAttribute( name.c_str(), &val );
    }
    return val;
  }

  double getDoubleElem(const string& name){  
    double val;
    XMLElement* tmp = this->current;
    while( string(tmp->Name()) != name ){
      tmp = tmp->NextSiblingElement();
    }
    tmp->QueryDoubleText( &val );
    return val;
  }

  double getIntAttr(const string& name, bool parent=false){  
    int val;
    if(parent){
      //XMLAttribute* at = this->root->FindAttribute( name.c_str() );
      this->root->QueryIntAttribute( name.c_str(), &val );
    }
    else{
      this->current->QueryIntAttribute( name.c_str(), &val );
    }
    return val;
  }

  double getIntElem(const string& name){  
    int val;
    XMLElement* tmp = this->current;
    while( string(tmp->Name()) != name ){
      tmp = tmp->NextSiblingElement();
    }
    tmp->QueryIntText(&val );
    return val;
  }


  bool ok(){
    XMLElement* tmp;
    bool ret;
    tmp = this->current->NextSiblingElement();
    if( tmp==NULL){
      ret = false;
    }
    else{
      this->current = tmp;
      ret = true;
    }
    return ret;
  }


protected:
  XMLDocument xml;
  XMLElement *root;
  XMLElement *current;
};




#endif
