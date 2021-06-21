#pragma once

#if defined(ARDUPILOT_BUILD)
#  undef _GLIBCXX_USE_C99_STDIO   // vsnprintf() not defind
#endif

#include <stdio.h>

#include <string>
#include <vector>
using std::string;
using std::vector;

#if defined(ARDUPILOT_BUILD)
#  pragma GCC optimize("O1")
#endif

#include "rapidjson/document.h"

#if defined(ARDUPILOT_BUILD)
#  pragma GCC reset_options
#endif
#include "rapidjson/pointer.h"
using namespace rapidjson;

//
// property system style interface with a rapidjson document as the backend
//

extern Document doc;

class PropertyNode
{
public:
    // Constructor.
    PropertyNode();
    PropertyNode(string abs_path, bool create=true);
    PropertyNode(Value *v);

    // Destructor.
    // ~PropertyNode();

    // operator =
    // PropertyNode & operator= (const PropertyNode &node);

    bool hasChild(const char *name );
    PropertyNode getChild( const char *name, bool create=true );
    PropertyNode getChild( const char *name, int index, bool create=true );

    bool isNull();		// return true if pObj pointer is NULL
    
    int getLen(); // return len if node is an array (else 0)
    void setLen(int size); // set array len of node, extend if necessary
    void setLen(int size, double init_val); // set array size, extend and init

    vector<string> getChildren(bool expand=true); // return list of children

    bool isLeaf( const char *name); // return true if pObj/name is leaf
    
    // value getters
    bool getBool( const char *name );	  // return value as a bool
    int getInt( const char *name );	  // return value as an int
    float getFloat( const char *name ); // return value as a float
    double getDouble( const char *name ); // return value as a double
    string getString( const char *name ); // return value as a string

    // indexed value getters
    bool getBool( const char *name, int index ); // return value as a bool
    int getInt( const char *name, int index ); // return value as a long
    float getFloat( const char *name, int index ); // return value as a double
    double getDouble( const char *name, int index ); // return value as a double
    string getString( const char *name, int index ); // return value as a string

    // value setters
    bool setBool( const char *name, bool val );     // returns true if successful
    bool setInt( const char *name, int n );     // returns true if successful
    bool setFloat( const char *name, float x ); // returns true if successful
    bool setDouble( const char *name, double x ); // returns true if successful
    bool setString( const char *name, string s ); // returns true if successful

    // indexed value setters
    bool setDouble( const char *name, int index, double val  ); // returns true if successful
    
    // void print();
    void pretty_print();

    Value *get_valptr() {  return val; }
    
private:
    // Pointer p;
    Value *val = nullptr;
};


// This function must be called before any rjPropertyNode usage. It
// imports the python props and props_xml/json modules.
extern void pyPropsInit();

// This function can be called at exit to properly free resources
// requested by init()
extern void pyPropsCleanup(void);

// Return a rjPropertyNode object that points to the specified path in
// the property tree.  This is a 'heavier' operation so it is
// recommended to call this function from initialization routines and
// save the result.  Then use the rjPropertyNode for direct read/write
// access in your update routines.
// extern PropertyNode GetNode(string abs_path, bool create=false);

// Read a json file and place the results at specified node
extern bool readJSON(string filename, PropertyNode *node);
    
// Write a json file beginning with the specified node
extern bool writeJSON(string filename, PropertyNode *node);