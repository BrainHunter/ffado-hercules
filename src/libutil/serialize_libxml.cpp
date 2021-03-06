/*
 * Copyright (C) 2005-2008 by Daniel Wagner
 *
 * This file is part of FFADO
 * FFADO = Free FireWire (pro-)audio drivers for Linux
 *
 * FFADO is based upon FreeBoB.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) version 3 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "version.h" // FOR CACHE_VERSION

#include "serialize.h"

#include <cstring>
#include <cstdio>
#include <cstdlib>

using namespace std;

IMPL_DEBUG_MODULE( Util::XMLSerialize,   XMLSerialize,   DEBUG_LEVEL_NORMAL );
IMPL_DEBUG_MODULE( Util::XMLDeserialize, XMLDeserialize, DEBUG_LEVEL_NORMAL );

Util::XMLSerialize::XMLSerialize( std::string fileName )
    : IOSerialize()
    , m_filepath( fileName )
    , m_verboseLevel( DEBUG_LEVEL_NORMAL )
{
    setDebugLevel( DEBUG_LEVEL_NORMAL );
    try {
        m_doc.create_root_node( "ffado_cache" );
        writeVersion();
    } catch ( const exception& ex ) {
        cout << "Exception caught: " << ex.what();
    }
}

Util::XMLSerialize::XMLSerialize( std::string fileName, int verboseLevel )
    : IOSerialize()
    , m_filepath( fileName )
    , m_verboseLevel( verboseLevel )
{
    setDebugLevel(verboseLevel);
    try {
        m_doc.create_root_node( "ffado_cache" );
        writeVersion();
    } catch ( const exception& ex ) {
        cout << "Exception caught: " << ex.what();
    }
}

Util::XMLSerialize::~XMLSerialize()
{
    try {
        m_doc.write_to_file_formatted( m_filepath );
    } catch ( const exception& ex ) {
        cout << "Exception caugth: " << ex.what();
    }

}

void
Util::XMLSerialize::writeVersion()
{
#if LIBXMLXX_MAJOR_VERSION == 3
    xmlpp::Element* pElem =  m_doc.get_root_node()->add_child_element( "CacheVersion" );
#else
    xmlpp::Node* pNode = m_doc.get_root_node();
    xmlpp::Element* pElem = pNode->add_child( "CacheVersion" );
#endif
    char* valstr;
    asprintf( &valstr, "%s", CACHE_VERSION );
#if LIBXMLXX_MAJOR_VERSION == 3
    pElem->set_first_child_text( valstr );
#else
    pElem->set_child_text( valstr );
#endif
    free( valstr );
}

bool
Util::XMLSerialize::write( std::string strMemberName,
                           long long value )

{
    debugOutput( DEBUG_LEVEL_VERY_VERBOSE, "write %s = %lld\n",
                 strMemberName.c_str(), value );

    vector<string> tokens;
    tokenize( strMemberName, tokens, "/" );

    if ( tokens.size() == 0 ) {
        debugWarning( "token size is 0\n" );
        return false;
    }

    xmlpp::Element* pNode = m_doc.get_root_node();
    pNode = getNodePath( pNode, tokens );

    // element to be added
#if LIBXMLXX_MAJOR_VERSION == 3
    xmlpp::Element* pElem = pNode->add_child_element( tokens[tokens.size() - 1] );
#else
    xmlpp::Element* pElem = pNode->add_child( tokens[tokens.size() - 1] );
#endif
    char* valstr;
    asprintf( &valstr, "%lld", value );
#if LIBXMLXX_MAJOR_VERSION == 3
    pElem->set_first_child_text( valstr );
#else
    pElem->set_child_text( valstr );
#endif
    free( valstr );

    return true;
}

bool
Util::XMLSerialize::write( std::string strMemberName,
                           std::string str)
{
    debugOutput( DEBUG_LEVEL_VERY_VERBOSE, "write %s = %s\n",
                 strMemberName.c_str(), str.c_str() );

    vector<string> tokens;
    tokenize( strMemberName, tokens, "/" );

    if ( tokens.size() == 0 ) {
        debugWarning( "token size is 0\n" );
        return false;
    }

    xmlpp::Element* pNode = m_doc.get_root_node();
    pNode = getNodePath( pNode, tokens );

    // element to be added
#if LIBXMLXX_MAJOR_VERSION == 3
    xmlpp::Element* pElem = pNode->add_child_element( tokens[tokens.size() - 1] );
    pElem->set_first_child_text( str );
#else
    xmlpp::Element* pElem = pNode->add_child( tokens[tokens.size() - 1] );
    pElem->set_child_text( str );
#endif

    return true;
}

xmlpp::Element*
Util::XMLSerialize::getNodePath( xmlpp::Element* pRootNode,
                                 std::vector<string>& tokens )
{
    // returns the correct node on which the new element has to be added.
    // if the path does not exist, it will be created.

    if ( tokens.size() == 1 ) {
        return pRootNode;
    }

    unsigned int iTokenIdx = 0;
    xmlpp::Element* pCurNode = pRootNode;
    for (bool bFound = false;
         ( iTokenIdx < tokens.size() - 1 );
         bFound = false, iTokenIdx++ )
    {
        xmlpp::Node::NodeList nodeList = pCurNode->get_children();
        for ( xmlpp::Node::NodeList::iterator it = nodeList.begin();
              it != nodeList.end();
              ++it )
        {
            if ( ( *it )->get_name() == tokens[iTokenIdx] ) {
                pCurNode = (xmlpp::Element*) *it;
                bFound = true;
                break;
            }
        }
        if ( !bFound ) {
            break;
        }
    }

    for ( unsigned int i = iTokenIdx; i < tokens.size() - 1; i++, iTokenIdx++ ) {
#if LIBXMLXX_MAJOR_VERSION == 3
        pCurNode = pCurNode->add_child_element( tokens[iTokenIdx] );
#else
        pCurNode = pCurNode->add_child( tokens[iTokenIdx] );
#endif
    }
    return pCurNode;

}

/***********************************/

Util::XMLDeserialize::XMLDeserialize( std::string fileName )
    : IODeserialize()
    , m_filepath( fileName )
    , m_verboseLevel( DEBUG_LEVEL_NORMAL )
{
    setDebugLevel(DEBUG_LEVEL_NORMAL);
    try {
        m_parser.set_substitute_entities(); //We just want the text to
                                            //be resolved/unescaped
                                            //automatically.
        m_parser.parse_file( m_filepath );
    } catch ( const exception& ex ) {
        cout << "Exception caught: " << ex.what();
    }
}

Util::XMLDeserialize::XMLDeserialize( std::string fileName, int verboseLevel )
    : IODeserialize()
    , m_filepath( fileName )
    , m_verboseLevel( verboseLevel )
{
    setDebugLevel(verboseLevel);
    try {
        m_parser.set_substitute_entities(); //We just want the text to
                                            //be resolved/unescaped
                                            //automatically.
        m_parser.parse_file( m_filepath );
    } catch ( const exception& ex ) {
        cout << "Exception caught: " << ex.what();
    }
}

Util::XMLDeserialize::~XMLDeserialize()
{
}

bool
Util::XMLDeserialize::isValid()
{
    return checkVersion();
}

bool
Util::XMLDeserialize::checkVersion()
{
    std::string savedVersion;
    if (read( "CacheVersion", savedVersion )) {
        Glib::ustring expectedVersion = CACHE_VERSION;
        debugOutput( DEBUG_LEVEL_NORMAL, "Cache version: %s, expected: %s.\n", savedVersion.c_str(), expectedVersion.c_str() );
        if (expectedVersion == savedVersion) {
            debugOutput( DEBUG_LEVEL_VERBOSE, "Cache version OK.\n" );
            return true;
        } else {
            debugOutput( DEBUG_LEVEL_VERBOSE, "Cache version not OK.\n" );
            return false;
        }
    } else return false;
}

bool
Util::XMLDeserialize::read( std::string strMemberName,
                            long long& value )

{
    debugOutput( DEBUG_LEVEL_VERY_VERBOSE, "lookup %s\n", strMemberName.c_str() );

    xmlpp::Document *pDoc=m_parser.get_document();
    if(!pDoc) {
        debugWarning( "no document found\n" );
        return false;
    }
    xmlpp::Node* pNode = pDoc->get_root_node();

    debugOutput( DEBUG_LEVEL_VERY_VERBOSE, "pNode = %s\n", pNode->get_name().c_str() );

#if LIBXMLXX_MAJOR_VERSION == 3
    xmlpp::Node::NodeSet nodeSet = pNode->find( strMemberName );
    for ( xmlpp::Node::NodeSet::iterator it = nodeSet.begin();
          it != nodeSet.end();
          ++it )
#else
    xmlpp::NodeSet nodeSet = pNode->find( strMemberName );
    for ( xmlpp::NodeSet::iterator it = nodeSet.begin();
          it != nodeSet.end();
          ++it )
#endif
    {
        const xmlpp::Element* pElement =
            dynamic_cast< const xmlpp::Element* >( *it );
        if ( pElement && pElement->has_child_text() ) {
            char* tail;
#if LIBXMLXX_MAJOR_VERSION == 3
            value = strtoll( pElement->get_first_child_text()->get_content().c_str(),
                             &tail, 0 );
#else
            value = strtoll( pElement->get_child_text()->get_content().c_str(),
                             &tail, 0 );
#endif
            debugOutput( DEBUG_LEVEL_VERY_VERBOSE, "found %s = %lld\n",
                         strMemberName.c_str(), value );
            return true;
        }
        debugWarning( "no such a node %s\n", strMemberName.c_str() );
        return false;
    }

    debugWarning( "no such a node %s\n", strMemberName.c_str() );
    return false;
}

bool
Util::XMLDeserialize::read( std::string strMemberName,
                            std::string& str )
{
    debugOutput( DEBUG_LEVEL_VERY_VERBOSE, "lookup %s\n", strMemberName.c_str() );

    xmlpp::Document *pDoc=m_parser.get_document();
    if(!pDoc) {
        debugWarning( "no document found\n" );
        return false;
    }
    xmlpp::Node* pNode = pDoc->get_root_node();

#if LIBXMLXX_MAJOR_VERSION == 3
    xmlpp::Node::NodeSet nodeSet = pNode->find( strMemberName );
    for ( xmlpp::Node::NodeSet::iterator it = nodeSet.begin();
          it != nodeSet.end();
          ++it )
#else
    xmlpp::NodeSet nodeSet = pNode->find( strMemberName );
    for ( xmlpp::NodeSet::iterator it = nodeSet.begin();
          it != nodeSet.end();
          ++it )
#endif
    {
        const xmlpp::Element* pElement = dynamic_cast< const xmlpp::Element* >( *it );
        if ( pElement ) {
            if ( pElement->has_child_text() ) {
#if LIBXMLXX_MAJOR_VERSION == 3
                str = pElement->get_first_child_text()->get_content();
#else
                str = pElement->get_child_text()->get_content();
#endif
            } else {
                str = "";
            }
            debugOutput( DEBUG_LEVEL_VERY_VERBOSE, "found %s = %s\n",
                         strMemberName.c_str(), str.c_str() );
            return true;
        }
        debugWarning( "no such a node %s\n", strMemberName.c_str() );
        return false;
    }

    debugWarning( "no such a node %s\n", strMemberName.c_str() );
    return false;
}

bool
Util::XMLDeserialize::isExisting( std::string strMemberName )
{
    xmlpp::Document *pDoc=m_parser.get_document();
    if(!pDoc) {
        return false;
    }
    xmlpp::Node* pNode = pDoc->get_root_node();
#if LIBXMLXX_MAJOR_VERSION == 3
    xmlpp::Node::NodeSet nodeSet = pNode->find( strMemberName );
#else
    xmlpp::NodeSet nodeSet = pNode->find( strMemberName );
#endif
    return nodeSet.size() > 0;
}

void
tokenize(const string& str,
         vector<string>& tokens,
         const string& delimiters)
{
    // Skip delimiters at beginning.
    string::size_type lastPos = str.find_first_not_of(delimiters, 0);
    // Find first "non-delimiter".
    string::size_type pos     = str.find_first_of(delimiters, lastPos);

    while (string::npos != pos || string::npos != lastPos)
    {
        // Found a token, add it to the vector.
        tokens.push_back(str.substr(lastPos, pos - lastPos));
        // Skip delimiters.  Note the "not_of"
        lastPos = str.find_first_not_of(delimiters, pos);
        // Find next "non-delimiter"
        pos = str.find_first_of(delimiters, lastPos);
    }
}
