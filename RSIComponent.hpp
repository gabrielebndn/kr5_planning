// Copyright  (C)  2009  Wilm Decre <wilm dot decre at mech dot kuleuven dot be>
// Copyright  (C)  2009  Ruben Smits <ruben dot smits at mech dot kuleuven dot be>,

// Author: Ruben Smits, Wilm Decre
// Maintainer: Ruben Smits, Wilm Decre

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

#ifndef _RSI_COMPONENT_HPP_
#define _RSI_COMPONENT_HPP_

#include <rtt/Component.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Logger.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/un.h>
#include <sys/types.h>
#include <netdb.h>

#include "../ticpp/ticpp.h"

#include <ocl/OCL.hpp>


namespace KR5 {
    
    class RSIComponent: public RTT::TaskContext {
    public:
        RSIComponent(const std::string& name);
        ~RSIComponent();
        
        virtual bool configureHook();
        virtual bool startHook() {
            return true;
        }
        virtual void updateHook();
        virtual void stopHook();
        virtual void cleanupHook();
        
        enum element_type {
            BOOL, STRING, LONG, FLOAT, DOUBLE, NONE
        };

        struct recv_element_base
        {
        public:
            std::string name;
            element_type type;
            ticpp::Element ti_el;
            recv_element_base(){}
            recv_element_base(const std::string& name_, element_type type_) :name(name_), type(type_){}

            virtual void updateElement(RTT::InputPort<std::vector<double> >* port){}
            virtual bool addAttribute(const std::string& attr)
            {
                return false;
            }
        };
        
        template<typename T> struct recv_element : public recv_element_base
        {
        public:
            T value;
            recv_element(){}
            recv_element(const std::string name_, element_type type_) : recv_element_base(name_, type_) {}

            void updateElement(RTT::InputPort<T>* port)
            {
               ti_el.SetText<T>(port->read(value));
            }
        };
        
        template<typename T> struct recv_element_vector: public recv_element_base {
            typedef T DataType;
            std::vector<T> value;
            std::vector<std::string> attributes;

            recv_element_vector(){}
            recv_element_vector(const std::string& name_, element_type type_):recv_element_base(name_, type_){}

            void updateElement(RTT::InputPort<std::vector<DataType> >* port){
                port->read(value);
                if(value.size() != attributes.size())
                    value.resize(attributes.size());
                for (unsigned int i=0;i<attributes.size();i++)
                    try{
                        #ifndef NDEBUG
                                RTT::log(RTT::Debug)<<"Updating "<<name<<"."<<attributes[i]<<RTT::endlog();
                        #endif
                                ti_el.SetAttribute<T> (attributes[i], value[i]);
                    } catch (ticpp::Exception ex) {
                        #ifndef NDEBUG
                                RTT::log(RTT::Error) << ex.what() << RTT::endlog();
                        #endif
                    }
            }

            virtual bool addAttribute(const std::string& attr)
            {
                //add attribute to list of attributes:
                attributes.push_back(attr);
                RTT::log(RTT::Debug)<<"Adding attribute "<<attr<<" to element "<<name<<RTT::endlog();
                return true;
            }
                    
        };
        
        struct element_base  {
        public:
            std::string name;
            element_type type;
            element_base() {}

            element_base(const std::string& name_, element_type type_) : name(name_), type(type_) {}

            virtual void updatePort(ticpp::Element* ti_el, RTT::OutputPort<std::vector<double> >* port){}
            virtual bool addAttribute(const std::string& attr)
            {
                return false;
            }
        };
        
        template<typename T> struct element: public element_base
        {
        public:
            T value;
            element() {}
            element(const std::string& name_, element_type type_) : element_base(name_, type_) {}

            void updatePort(ticpp::Element* ti_el, RTT::OutputPort<T>* port)
            {
                ti_el->GetText<T> (&value);
                port->write(value);
            }
        };


        template<typename T>  struct element_vector: public element_base
        {
        public:
            typedef T DataType;
            std::vector<DataType> value;
            std::vector<std::string> attributes;

            element_vector(){}

            element_vector(const std::string& name_, element_type type_) : element_base(name_, type_)  {}

            void updatePort(ticpp::Element* ti_el, RTT::OutputPort<std::vector<DataType> >* port)
            {
                if (value.size() != attributes.size())
                    value.resize(attributes.size());
                for (unsigned int i = 0; i < attributes.size(); i++)
                    try {
                        #ifndef NDEBUG
                                RTT::log(RTT::Debug)<<"Updating "<<name<<"."<<attributes[i]<<RTT::endlog();
                        #endif
                                ti_el->GetAttribute<DataType> (attributes[i], &(value[i]));
                    } catch (ticpp::Exception ex) {
                        #ifndef NDEBUG
                                RTT::log(RTT::Debug) << ex.what() << RTT::endlog();
                        #endif
                    }

               port->write(value);
            }
            virtual bool addAttribute(const std::string& attr)
            {
                 //add attribute to list of attributes:
                 attributes.push_back(attr);
                 RTT::log(RTT::Debug)<<"Adding attribute "<<attr<<" to element "<<name<<RTT::endlog();
                 return true;
             }
        };
        
    private:
        RTT::Property<std::string> ERXconfig;
        int serversock, clientsock;
        struct sockaddr_in localserver;
        struct sockaddr_in localclient;
        struct sockaddr_un udpclient;
        char* buffer;
        
        std::string sentype, protocol;
        bool protocol_is_udp;
        int port;
        long long ipoc;
        bool only_send;
        bool connected;
        bool message_received;
        std::vector<element_base*> all_send_elements;
        std::vector<recv_element_base*> all_recv_elements;
        
        std::map<std::string, element_vector<double> > internal_elements;

        std::string bufferstring;
        ticpp::Element* rob;
        ticpp::Element* ipoc_el;
        ticpp::Element ipoc_recv;
        
        ticpp::Document doc_buffer;
        ticpp::Element recv_message;
        element_type fromStringToType(const std::string& type);
        std::string fromTypeToString(const element_type& el);
        
        bool parseConfig();
        bool prepareMessage();
        std::ostringstream sendbuffer;

        RTT::OutputPort<std::vector<double> > send_RIst;
        RTT::OutputPort<std::vector<double> > send_RSol;
        RTT::OutputPort<std::vector<double> > send_AIPos;
        RTT::OutputPort<std::vector<double> > send_ASPos;
        RTT::OutputPort<std::vector<double> > send_MACur;
        RTT::OutputPort<std::vector<double> > send_Delay;

        RTT::InputPort<std::string > recv_EStr;
        RTT::InputPort<std::vector<double> > recv_RKorr;
        RTT::InputPort<std::vector<double> > recv_AKorr;
        
    };
    
}//namespace KR5

#endif //_KR5_RSI_COMPONENT_HPP_
