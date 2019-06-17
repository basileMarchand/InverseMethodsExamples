#ifndef _STRING_UTILITIES_H_
#define _STRING_UTILITIES_H_

#include <string>
#include <algorithm>
#include <iostream>

using namespace std;


template<typename T>
struct STRANS;


class STRUTIL{
public:
  static bool startwith(string token, string pattern){
    bool ret = false;
    if( token.substr( 0, pattern.size()) == pattern){
      ret = true;
    }
    return ret;
  }
  
  static bool endwith(string token, string pattern){
    bool ret=false;
    if( token.substr( token.size()-pattern.size(), pattern.size()) == pattern){
      ret = true;
    }
    return ret;
  }
  
  static void _erase_space(string& the_str){
    string pattern = " ";
    bool space_before = STRUTIL::startwith(the_str, pattern);
    bool space_after = STRUTIL::endwith(the_str, pattern);
    while( space_before || space_after ){
      space_before = STRUTIL::startwith(the_str, pattern);
      if( space_before ){
	the_str = the_str.substr(1,the_str.size()-1);
      }
      space_after = STRUTIL::endwith(the_str, pattern);
      if( space_after ){
	the_str = the_str.substr(0,the_str.size()-1);
      }
    }
  }
  
  static string format(const int& x, int n_zero=4){
    return string( n_zero - to_string(x).size(), '0') + to_string(x);
  } 

  static vector<string> tokenize(string x, string delimiter){
    size_t pos = 0;
    string token;
    vector<string> res(0);
    while ((pos = x.find(delimiter)) != string::npos) {
      token = x.substr(0, pos);
      res.push_back( token );
      x.erase(0, pos + delimiter.length());
    }
    res.push_back(x);
    return res;
  }

  template<typename T>
  static vector<T> Tokenize(string x, string delimiter){
    STRUTIL::_erase_space(x);
    size_t pos = 0;
    string token;
    vector<T> res(0);
    while ((pos = x.find(delimiter)) != string::npos) {
      token = x.substr(0, pos);
      T v = STRANS<T>::convert(token);
      res.push_back( v );
      x.erase(0, pos + delimiter.length());
      STRUTIL::_erase_space(x);
    }
    if( x != "" && x != delimiter ){
      res.push_back( STRANS<T>::convert(x) );
    }
    return res;
  }
  
};




template<typename T>
struct STRANS{ 
  static T convert(string x){}
  static T convert(int x){}
  static T convert(size_t x){}
  static T convert(double x){}
};


template<> 
struct STRANS<int>{ static int convert(string x){
    return stoi(x);
  }
};

template<> 
struct STRANS<double>{ 
  static double convert(string x){
    double v=0;
    sscanf(x.c_str(), "%lf",&v);
    return v;
  }
};

template<> 
struct STRANS<string>{ 
  static string convert(string x){
    return x;
  }
  
  static string convert(int x){
    return std::to_string(x);
  }

  static string convert(size_t x){
    return std::to_string(x);
  }

  static string convert(double x){
    return std::to_string(x);
  }



};








#endif
