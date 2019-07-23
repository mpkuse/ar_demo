#pragma once

#include <iostream>
#include <string>
#include <vector>



class MiscUtils {
public:




    static std::vector<std::string>
    split( std::string const& original, char separator )
    {
        std::vector<std::string> results;
        std::string::const_iterator start = original.begin();
        std::string::const_iterator end = original.end();
        std::string::const_iterator next = std::find( start, end, separator );
        while ( next != end ) {
            results.push_back( std::string( start, next ) );
            start = next + 1;
            next = std::find( start, end, separator );
        }
        results.push_back( std::string( start, next ) );
        return results;
    }


};
