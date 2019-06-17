#include "XmlTree.hpp"

XmlBlock::XmlBlock(){

}

XmlBlock::XmlBlock(XMLElement* elem){
  this->root = elem;
  this->current = elem->FirstChildElement();
}

XmlBlock::XmlBlock( XmlBlock& other){
  this->root = other.root;
  this->current = other.current;
}

void XmlBlock::fromFile(const string& fname){
  XMLError out = this->xml.LoadFile( fname.c_str() );
  CHECK_XML( out );
  this->root = this->xml.RootElement();
  this->current = this->root->FirstChildElement();
}

vector<string> XmlBlock::listRoot(){
   vector<string> blocks_id(0);
   XMLElement* elem = this->root;
   while( elem != NULL ){
     string name(elem->Name());
     blocks_id.push_back(name);
     elem = elem->NextSiblingElement();
   }
   return blocks_id;
}


vector<string> XmlBlock::listBlocks(){
   vector<string> blocks_id(0);
   XMLElement* elem = this->current;
   while( elem != NULL ){
     string name(elem->Name());
     blocks_id.push_back(name);
     elem = elem->NextSiblingElement();
   }
   return blocks_id;
}

XmlBlock* XmlBlock::getBlock( string block_tag, int offset){
  XMLElement* elem = this->current;
  int count=0;
   while( elem != NULL ){
     string name(elem->Name());
     if( name == block_tag  ){
       break;
     }
     elem = elem->NextSiblingElement();
   }
   this->current = elem->NextSiblingElement();

   XmlBlock* res = new XmlBlock( elem );
   return res;
}


bool XmlBlock::elemExists(const string& ename){
  XMLElement* elem = this->current;
  bool found=false;
  while( elem != NULL ){
    string name(elem->Name());
    if( name == ename ){
      found = true;
      break;
    }
    elem = elem->NextSiblingElement();
  }
  return found;
}




