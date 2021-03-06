/*
 * Copyright (C) 2005-2008 by Pieter Palmers
 * Copyright (C) 2005-2008 by Daniel Wagner
 *
 * This file is part of FFADO
 * FFADO = Free FireWire (pro-)audio drivers for Linux
 *
 * FFADO is based upon FreeBoB
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

#ifndef AVC_PLUG
#define AVC_PLUG

#include "../ccm/avc_signal_source.h"
#include "../streamformat/avc_extended_stream_format.h"
#include "../avc_definitions.h"
#include "avc_generic.h"

#include "libutil/serialize.h"

#include "debugmodule/debugmodule.h"

#include <string>

class Ieee1394Service;
class ConfigRom;

namespace AVC {

class Unit;
class Subunit;
class PlugManager;

class Plug;
typedef std::vector<Plug*> PlugVector;

class PlugConnection;
typedef std::vector<PlugConnection*> PlugConnectionVector;


class Plug {
public:

    enum EPlugAddressType {
        eAPA_PCR,
        eAPA_ExternalPlug,
        eAPA_AsynchronousPlug,
        eAPA_SubunitPlug,
        eAPA_FunctionBlockPlug,
        eAPA_Undefined,
    };
    std::string plugAddressTypeToString(enum EPlugAddressType t);

    enum EPlugType {
        eAPT_IsoStream,
        eAPT_AsyncStream,
        eAPT_Midi,
        eAPT_Sync,
        eAPT_Analog,
        eAPT_Digital,
        eAPT_Unknown,
    };
    std::string plugTypeToString(enum EPlugType t);

    enum EPlugDirection {
        eAPD_Input,
        eAPD_Output,
        eAPD_Unknown,
    };
    std::string plugDirectionToString(enum EPlugDirection t);

    // \todo This constructors sucks. too many parameters. fix it.
    Plug( Unit* unit,
          Subunit* subunit,
          function_block_type_t functionBlockType,
          function_block_type_t functionBlockId,
          EPlugAddressType plugAddressType,
          EPlugDirection plugDirection,
          plug_id_t plugId );
    Plug( Unit* unit,
          Subunit* subunit,
          function_block_type_t functionBlockType,
          function_block_type_t functionBlockId,
          EPlugAddressType plugAddressType,
          EPlugDirection plugDirection,
          plug_id_t plugId,
          int globalId );
    Plug( const Plug& rhs );
    virtual ~Plug();

    bool inquireConnnection( Plug& plug );
    bool setConnection( Plug& plug );

    int getSignalSource();

    int getGlobalId() const
        { return m_globalId; }
    plug_id_t getPlugId() const
        { return m_id; }
    ESubunitType getSubunitType() const;
    subunit_id_t getSubunitId() const;

    const char* getName() const
        { return m_name.c_str(); }
    bool setName(const char *n)
        { m_name=n; return true; }
    bool setName(std::string n)
        { m_name=n; return true; }
    EPlugDirection getPlugDirection() const
        { return m_direction; }
    bool setPlugDirection(EPlugDirection d)
        { m_direction=d; return true; }
    sampling_frequency_t getSamplingFrequency() const
        { return m_samplingFrequency; }
    int getSampleRate() const; // 22050, 24000, 32000, ...
    bool setSampleRate( int rate );
    bool supportsSampleRate( int rate );

    int getNrOfChannels() const;
    bool setNrOfChannels(int i);
    int getNrOfStreams() const;

    // FIXME: this is the same as getPlugDirection
    EPlugDirection getDirection() const
        { return m_direction; }
    EPlugAddressType getPlugAddressType() const
        { return m_addressType; }
    EPlugType getPlugType() const
        { return m_infoPlugType; }
    bool setPlugType(EPlugType t)
        { m_infoPlugType=t; return true; }

    function_block_type_t getFunctionBlockType() const
        { return m_functionBlockType; }
    function_block_id_t getFunctionBlockId() const
        { return m_functionBlockId; }

//     const PlugVector& getInputConnections() const
//         { return m_inputConnections; }
//     const PlugVector& getOutputConnections() const
//         { return m_outputConnections; }
    PlugVector& getInputConnections()
        { return m_inputConnections; }
    PlugVector& getOutputConnections()
        { return m_outputConnections; }

    static PlugAddress::EPlugDirection convertPlugDirection(
    EPlugDirection direction);

    void showPlug() const;

    bool serialize( std::string basePath, Util::IOSerialize& ser ) const;
    static Plug* deserialize( std::string basePath,
                              Util::IODeserialize& deser,
                              Unit& avDevice,
                              PlugManager& plugManager );
    bool deserializeConnections( std::string basePath, 
                                 Util::IODeserialize& deser );
    bool deserializeUpdateSubunit();

 public:
    struct ChannelInfo {
        stream_position_t          m_streamPosition;
        stream_position_location_t m_location;
        std::string              m_name;
    };
    typedef std::vector<ChannelInfo> ChannelInfoVector;

    struct ClusterInfo {
        int                 m_index;
        port_type_t         m_portType;
        std::string         m_name;

        nr_of_channels_t    m_nrOfChannels;
        ChannelInfoVector   m_channelInfos;
        stream_format_t     m_streamFormat;
        int                 m_buildSource; // To track how we are instantiated
    };
    typedef std::vector<ClusterInfo> ClusterInfoVector;

    ClusterInfoVector& getClusterInfos()
        { return m_clusterInfos; }

    virtual void setVerboseLevel( int i );

// the discovery interface
// everything can be overloaded, or
// can be left as is
public:
    virtual bool discover();
    virtual bool discoverConnections();
    virtual bool propagateFromConnectedPlug( );

protected:
    virtual bool discoverPlugType();
    virtual bool discoverName();
    virtual bool discoverNoOfChannels();
    virtual bool discoverChannelPosition();
    virtual bool discoverChannelName();
    virtual bool discoverClusterInfo();
    virtual bool discoverStreamFormat();
    virtual bool discoverSupportedStreamFormats();
    virtual bool discoverConnectionsInput();
    virtual bool discoverConnectionsOutput();
    virtual bool initFromDescriptor();

    bool propagateFromPlug( Plug *p );

    ExtendedStreamFormatCmd setPlugAddrToStreamFormatCmd(
            ExtendedStreamFormatCmd::ESubFunction subFunction);

protected:
    Plug();

    SignalSourceCmd setSrcPlugAddrToSignalCmd();

    void setDestPlugAddrToSignalCmd(
            SignalSourceCmd& signalSourceCmd, Plug& plug );

    void debugOutputClusterInfos( int debugLevel );

    bool addPlugConnection( PlugVector& connections, Plug& plug );

    bool discoverConnectionsFromSpecificData(
        EPlugDirection discoverDirection,
        PlugAddressSpecificData* plugAddress,
        PlugVector& connections );

    Plug* getPlugDefinedBySpecificData(
        UnitPlugSpecificDataPlugAddress* pUnitPlugAddress,
        SubunitPlugSpecificDataPlugAddress* pSubunitPlugAddress,
        FunctionBlockPlugSpecificDataPlugAddress* pFunctionBlockPlugAddress );

    EPlugDirection toggleDirection( EPlugDirection direction ) const;

    const ClusterInfo* getClusterInfoByIndex( int index ) const;

    bool serializeChannelInfos( std::string basePath,
                                Util::IOSerialize& ser,
                                const ClusterInfo& clusterInfo ) const;
    bool deserializeChannelInfos( std::string basePath,
                                  Util::IODeserialize& deser,
                                  ClusterInfo& clusterInfo );

    bool serializeClusterInfos( std::string basePath,
                                Util::IOSerialize& ser ) const;
    bool deserializeClusterInfos( std::string basePath,
                                  Util::IODeserialize& deser );

    bool serializeFormatInfos( std::string basePath,
                               Util::IOSerialize& ser ) const;
    bool deserializeFormatInfos( std::string basePath,
                                 Util::IODeserialize& deser );

protected:
    // Supported stream formats
    struct FormatInfo {
        FormatInfo()
            : m_samplingFrequency( eSF_DontCare )
            , m_isSyncStream( false )
            , m_audioChannels( 0 )
            , m_midiChannels( 0 )
            , m_index( 0xff )
            {}
        sampling_frequency_t  m_samplingFrequency;
        bool                       m_isSyncStream;
        number_of_channels_t  m_audioChannels;
        number_of_channels_t  m_midiChannels;
        byte_t                     m_index;
    };
    typedef std::vector<FormatInfo> FormatInfoVector;


    Unit*                       m_unit;
    Subunit*                    m_subunit;
    ESubunitType                m_subunitType;
    subunit_t                   m_subunitId;
    function_block_type_t       m_functionBlockType;
    function_block_id_t         m_functionBlockId;
    EPlugAddressType            m_addressType;
    EPlugDirection              m_direction;
    plug_id_t                   m_id;
    EPlugType                   m_infoPlugType;
    nr_of_channels_t            m_nrOfChannels;
    std::string                 m_name;
    ClusterInfoVector           m_clusterInfos;
    sampling_frequency_t        m_samplingFrequency;
    FormatInfoVector            m_formatInfos;
    PlugVector                  m_inputConnections;
    PlugVector                  m_outputConnections;
    int                         m_globalId;

    DECLARE_DEBUG_MODULE;
};

const char* avPlugAddressTypeToString( Plug::EPlugAddressType addressType );
const char* avPlugTypeToString( Plug::EPlugType type );
const char* avPlugDirectionToString( Plug::EPlugDirection direction );

class PlugManager
{
public:
    PlugManager( );
    PlugManager( const PlugManager& rhs );
    ~PlugManager();

    bool addPlug( Plug& plug );
    bool remPlug( Plug& plug );

    void showPlugs() const;

    int getPlugCount()
        { return m_plugs.size(); };

    int requestNewGlobalId()
        { return m_globalIdCounter++; };

    Plug* getPlug( ESubunitType subunitType,
                   subunit_id_t subunitId,
                   function_block_type_t functionBlockType,
                   function_block_id_t functionBlockId,
                   Plug::EPlugAddressType plugAddressType,
                   Plug::EPlugDirection plugDirection,
                   plug_id_t plugId ) const;
    Plug* getPlug( int iGlobalId ) const;
    PlugVector getPlugsByType( ESubunitType subunitType,
			       subunit_id_t subunitId,
			       function_block_type_t functionBlockType,
			       function_block_id_t functionBlockId,
			       Plug::EPlugAddressType plugAddressType,
			       Plug::EPlugDirection plugDirection,
			       Plug::EPlugType type) const;

    bool serialize( std::string basePath, Util::IOSerialize& ser ) const;
    static  PlugManager* deserialize( std::string basePath,
				      Util::IODeserialize& deser,
				      Unit& avDevice );
    void setVerboseLevel( int i );
    PlugVector& getPlugs()
	{ return m_plugs; }
    bool tidyPlugConnections(PlugConnectionVector&);

    bool deserializeUpdate( std::string basePath,  
                            Util::IODeserialize& deser );

private:
    int m_globalIdCounter;

    PlugVector   m_plugs;

    DECLARE_DEBUG_MODULE;
};

class PlugConnection {
public:
    PlugConnection( Plug& srcPlug, Plug& destPlug );

    Plug& getSrcPlug() const
        { return *m_srcPlug; }
    Plug& getDestPlug() const
        { return *m_destPlug; }

    bool serialize( std::string basePath, Util::IOSerialize& ser ) const;
    static PlugConnection* deserialize( std::string basePath,
					Util::IODeserialize& deser,
					Unit& avDevice );
private:
    PlugConnection();

private:
    Plug* m_srcPlug;
    Plug* m_destPlug;
};

typedef std::vector<PlugConnection> PlugConnectionOwnerVector;


bool serializePlugVector( std::string basePath,
                          Util::IOSerialize& ser,
                          const PlugVector& vec);

bool deserializePlugVector( std::string basePath,
                            Util::IODeserialize& deser,
                            const PlugManager& plugManager,
                            PlugVector& vec );

}

#endif // AVC_PLUG
