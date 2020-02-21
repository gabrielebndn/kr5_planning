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


#include <rtt/Logger.hpp>

#include <netinet/in.h>
#include <sys/types.h>
#include <errno.h>
#include <fcntl.h>
#include <string>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <sstream>

#include "RSIComponent.hpp"

namespace KR5 {

using std::cout;
using std::endl;
using std::string;
using std::vector;
using std::map;
    using namespace RTT;
    
    RSIComponent::RSIComponent(const string& name) :
        TaskContext(name, PreOperational), ERXconfig("ERXconfig", "ERX-RSI configuration file", "ERXconfigOrocos.xml"), connected(false), message_received(false) {
            
            //TODO:Is this big enough? or should we make it configurable?
            buffer = new char[1024];
            
            // Create structures for internal element types:
            
            /*
         RIst  : Cartesian actual position
         RSol  : Cartesian command position
         AIPos : Axis-specific actual position (A1-A6)
         ASPos : Axis-specific command position (A1-A6)
         MACur : Axis-specific motor current (A1-A6)
         Delay : Number of late data packages
             */

            internal_elements["DEF_RIst"] = element_vector<double> ("RIst", DOUBLE);
            internal_elements["DEF_RIst"].attributes.push_back("X");
            internal_elements["DEF_RIst"].attributes.push_back("Y");
            internal_elements["DEF_RIst"].attributes.push_back("Z");
            internal_elements["DEF_RIst"].attributes.push_back("A");
            internal_elements["DEF_RIst"].attributes.push_back("B");
            internal_elements["DEF_RIst"].attributes.push_back("C");

            internal_elements["DEF_RSol"] = element_vector<double> ("RSol", DOUBLE);
            internal_elements["DEF_RSol"].attributes.push_back("X");
            internal_elements["DEF_RSol"].attributes.push_back("Y");
            internal_elements["DEF_RSol"].attributes.push_back("Z");
            internal_elements["DEF_RSol"].attributes.push_back("A");
            internal_elements["DEF_RSol"].attributes.push_back("B");
            internal_elements["DEF_RSol"].attributes.push_back("C");
            
            internal_elements["DEF_AIPos"] = element_vector<double> ("AIPos", DOUBLE);
            internal_elements["DEF_AIPos"].attributes.push_back("A1");
            internal_elements["DEF_AIPos"].attributes.push_back("A2");
            internal_elements["DEF_AIPos"].attributes.push_back("A3");
            internal_elements["DEF_AIPos"].attributes.push_back("A4");
            internal_elements["DEF_AIPos"].attributes.push_back("A5");
            internal_elements["DEF_AIPos"].attributes.push_back("A6");
            
            internal_elements["DEF_ASPos"] = element_vector<double> ("ASPos", DOUBLE);
            internal_elements["DEF_ASPos"].attributes.push_back("A1");
            internal_elements["DEF_ASPos"].attributes.push_back("A2");
            internal_elements["DEF_ASPos"].attributes.push_back("A3");
            internal_elements["DEF_ASPos"].attributes.push_back("A4");
            internal_elements["DEF_ASPos"].attributes.push_back("A5");
            internal_elements["DEF_ASPos"].attributes.push_back("A6");

            internal_elements["DEF_MACur"] = element_vector<double> ("MACur", DOUBLE);
            internal_elements["DEF_MACur"].attributes.push_back("A1");
            internal_elements["DEF_MACur"].attributes.push_back("A2");
            internal_elements["DEF_MACur"].attributes.push_back("A3");
            internal_elements["DEF_MACur"].attributes.push_back("A4");
            internal_elements["DEF_MACur"].attributes.push_back("A5");
            internal_elements["DEF_MACur"].attributes.push_back("A6");
            
            internal_elements["DEF_Delay"] = element_vector<double> ("Delay", DOUBLE);
            internal_elements["DEF_Delay"].attributes.push_back("D");

            //
            // Technology parameters not implemented yet
            //

            //Parse ERX configuration file
            if (!parseConfig())
                log(Critical)<<"Parsing ERXConfigOrocos.xml failed."<<endlog();

            //Register all necessary port

            for (vector<element_base*>::iterator it = all_send_elements.begin(); it != all_send_elements.end(); ++it)
            {
                std::string name;
                name = (*it)->name;
                if( name == "RIst")
                    addPort("RIst",send_RIst);
                else if( name == "RSol")
                    addPort("RSol",send_RSol);
                else if( name == "AIPos")
                    addPort("AIPos",send_AIPos);
                else if( name == "ASPos")
                    addPort("ASPos",send_ASPos);
                else if( name == "MACur")
                    addPort("MACur",send_MACur);
                else if( name == "Delay")
                    addPort("Delay",send_Delay);

            }

            for (vector<recv_element_base*>::iterator it = all_recv_elements.begin(); it != all_recv_elements.end(); ++it)
            {
                std::string name;
                name = (*it)->name;
                if( name == "DEF_EStr")
                    addPort("DEF_EStr",recv_EStr);
                else if( name == "RKorr")
                    addPort("RKorr",recv_RKorr);
                else if( name == "AKorr")
                    addPort("AKorr",recv_AKorr);
            }
            
            
        }
        
        RSIComponent::~RSIComponent() {
            delete buffer;
            for (unsigned int i = 0; i<all_recv_elements.size();i++)
                delete all_recv_elements[i];
            for (unsigned int i = 0; i<all_send_elements.size();i++)
                delete all_send_elements[i];
        }
        
        RSIComponent::element_type RSIComponent::fromStringToType(
                const string& type) {
            if (type == "BOOL")
                return BOOL;
            else if (type == "STRING")
                return STRING;
            else if (type == "LONG")
                return LONG;
            else if (type == "FLOAT")
                return FLOAT;
            else if (type == "DOUBLE")
                return DOUBLE;
            else
                return NONE;
        }
        
        string RSIComponent::fromTypeToString(const RSIComponent::element_type& el) {
            if (el == BOOL) {
                //log(Debug)<<"el==BOOL"<<endlog();
                return "BOOL";
            }
            else if (el == STRING) {
                //log(Debug)<<"el==STRING"<<endlog();
                return "STRING";
            }
            else if (el == LONG) {
                //log(Debug)<<"el==LONG"<<endlog();
                return "LONG";
            }
            else if (el == FLOAT) {
                //log(Debug)<<"el==FLOAT"<<endlog();
                return "FLOAT";
            }
            else if (el == DOUBLE) {
                //log(Debug)<<"el==DOUBLE"<<endlog();
                return "DOUBLE";
            }
            else
                return "NONE";
        }
        bool RSIComponent::parseConfig() {
            //Parse ERXConfig.xml
            try {
                ticpp::Document doc(ERXconfig.rvalue());
                doc.LoadFile();
                //Get the root node:
                ticpp::Element* root = doc.FirstChildElement("ROOT");
                //Get the CONFIG element
                ticpp::Element* config = root->FirstChildElement("CONFIG");
                //Parse the CONFIG element
                {
                    //Get the port number
                    ticpp::Element* port_el = config->FirstChildElement("PORT");
                    port_el->GetText<int>(&port);
                    log(Info) << "RSI communication through port: " << port << endlog();
                    //Get the protocol
                    ticpp::Element* protocol_el = config->FirstChildElement("PROTOCOL");
                    protocol_el->GetText<string>(&protocol);
                    log(Info) << "RSI communication uses protocol: " << protocol << endlog();
                    //Get the sentype
                    ticpp::Element* sentype_el = config->FirstChildElement("SENTYPE");
                    sentype_el->GetText<string>(&sentype);
                    log(Info) << "RSI communication uses sentype name: " << sentype << endlog();
                    //Get the only_send value
                    ticpp::Element* only_send_el = config->FirstChildElement("ONLYSEND");
                    only_send = ("TRUE" == only_send_el->GetText());
                    if (only_send)
                        log(Info) << "RSI communication will only send" << endlog();
                    else
                        log(Info) << "RSI communication is send and receive" << endlog();
                }
                
                //Get the SEND element
                ticpp::Element* send = root->FirstChildElement("SEND"); {
                    //Get the elements
                    ticpp::Element* all_elements = send->FirstChildElement("ELEMENTS");
                    //Iterate over all elements:
                    ticpp::Iterator<ticpp::Element> element_it("ELEMENT");
                    for (element_it = element_it.begin(all_elements); element_it
                            != element_it.end(); element_it++) {
                        //Get type:
                        const string& typestr = element_it->GetAttribute<string> ("TYPE");
                        //Convert:
                        element_type type = fromStringToType(typestr);
                        //log(Debug)<<"element type: "<<typestr<<endlog();
                        //log(Debug)<<"element type: "<<fromTypeToString(type)<<endlog();
                        if (type == NONE) {
                            log(Error) << "Type " << typestr << "not supported." << endlog();
                            return false;
                        }
                        
                        //Check value if TAG is internally defined:
                        const string& TAG = element_it->GetAttribute<std::string> ("TAG");
                        map<string, element_vector<double> >::const_iterator it_element = internal_elements.find(TAG);
                        if (it_element != internal_elements.end()) {
                            //element is internally defined
                            all_send_elements.push_back(new element_vector<double> (it_element->second));
                            log(Info) << "Adding internally defined element "
                                    << it_element->second.name << " of type "
                                    << fromTypeToString(it_element->second.type)
                                    << endlog();
                        }
                        else {
    						//Check if name contains a . if so, this element will have multiple attributes
    						string::size_type dotpos = TAG.find('.');
    						if (dotpos == string::npos)
    						{
    							//No dot found, simple element
    							switch (type)
    							{
    							case BOOL:
    								all_send_elements.push_back(new element<bool> (TAG, type));
    								break;
    							case STRING:
    								all_send_elements.push_back(new element<std::string> (TAG, type));
    								break;
    							case LONG:
    								all_send_elements.push_back(new element<int> (TAG, type));
    								break;
    							case FLOAT:
    								all_send_elements.push_back(new element<float> (TAG, type));
    								break;
    							case DOUBLE:
    								all_send_elements.push_back(new element<double> (TAG, type));
    								break;
    							case NONE:
    								log(Error) << "NONE as element type" << endlog();
    								break;
    							default:
    								log(Error) << "unknown element type" << endlog();
    								break;
    							}

    							log(Info) << "Adding element " << TAG << endlog();
    						} else
    						{
    							//Break the string into to parts:
    							const string& name = TAG.substr(0, dotpos);
    							const string& attribute = TAG.substr(dotpos + 1);
    							log(Debug) << "name: " << name << " attribute: " << attribute << endlog();
    							//check if there already exists an element with name
    							vector<element_base*>::iterator it;
    							for (it = all_send_elements.begin(); it != all_send_elements.end(); it++)
    							{
    								//log(Debug)<<"iterator name: "<<(*it)->name<<endlog();
    								if ((*it)->name == name)
    								{
    									if (!(*it)->addAttribute(attribute))
    									{
    										log(Error) << "Adding attribute " << attribute << "to element " << (*it)->name
    												<< "failed." << endlog();
    										return false;
    									}
    									break;
    								}
    							}
    							if (it == all_send_elements.end())
    							{
    								//New element
    								switch (type)
    								{
    								case BOOL:
    									all_send_elements.push_back(new element_vector<bool> (name, type));
    									break;
    								case STRING:
    									all_send_elements.push_back(new element_vector<string> (name, type));
    									break;
    								case LONG:
    									all_send_elements.push_back(new element_vector<int> (name, type));
    									break;
    								case FLOAT:
    									all_send_elements.push_back(new element_vector<float> (name, type));
    									break;
    								case DOUBLE:
    									all_send_elements.push_back(new element_vector<double> (name, type));
    									break;
    								case NONE:
    									log(Error) << "NONE as element type" << endlog();
    									break;
    								default:
    									log(Error) << "unknown element type" << endlog();
    									break;
    								}
    								if (!all_send_elements.back()->addAttribute(attribute))
    								{
    									log(Error) << "This should not occur" << endlog();
    									return false;
    								}
    								log(Info) << "Adding new element " << name << " with attribute " << attribute
    										<< endlog();
    								log(Debug) << "element name " << all_send_elements.back()->name << " of type "
    										<< fromTypeToString(all_send_elements.back()->type) << endlog();
    							}
    						}


                        }
                    }
                }

                vector<element_base*>::iterator it;
                for (it = all_send_elements.begin(); it != all_send_elements.end(); it++)
                {
                    log(Info) << "Name" << (*it)->name << endlog();
                }

                //Get the RECEIVE element
                ticpp::Element* receive = root->FirstChildElement("RECEIVE"); {
                    //Get the elements
                    ticpp::Element* all_elements = receive->FirstChildElement("ELEMENTS");
                    ticpp::Iterator<ticpp::Element> element_it("ELEMENT");
                    for (element_it = element_it.begin(all_elements); element_it
                            != element_it.end(); element_it++) {
                        //Get type:
                        const string& typestr = element_it->GetAttribute<string>("TYPE");
                        //Convert:
                        element_type type = fromStringToType(typestr);
                        //log(Debug)<<"element type: "<<typestr<<endlog();
                        //log(Debug)<<"element type: "<<fromTypeToString(type)<<endlog();
                        if (type == NONE) {
                            log(Error) << "Type " << typestr << "not supported." << endlog();
                            return false;
                        }
                        
                        //Check value if TAG is internally defined:
                        const string& TAG = element_it->GetAttribute<std::string> ("TAG");
                        //Check if name contains a . if so, this element will have multiple attributes
                        string::size_type dotpos = TAG.find('.');
                        if(dotpos==string::npos){
                            //No dot found, simple element
                            switch (type) {
                                case BOOL:
                                    all_recv_elements.push_back(new recv_element<bool>(TAG, type));
                                    break;
                                case STRING:
                                    all_recv_elements.push_back(new recv_element<std::string>(TAG, type));
                                    break;
                                case LONG:
                                    all_recv_elements.push_back(new recv_element<int>(TAG, type));
                                    break;
                                case FLOAT:
                                    all_recv_elements.push_back(new recv_element<float>(TAG, type));
                                    break;
                                case DOUBLE:
                                    all_recv_elements.push_back(new recv_element<double>(TAG, type));
                                    break;
                                case NONE:
                                    log(Error)<<"NONE as element type"<<endlog();
                                    break;
                                default:
                                    log(Error)<<"unknown element type"<<endlog();
                                    break;
                            }
                            
                            log(Info)<<"Adding rec element "<< TAG<<endlog();
                        }else{
                            //Break the string into to parts:
                            const string& name = TAG.substr(0, dotpos);
                            const string& attribute = TAG.substr(dotpos+1);
                            log(Info)<<"name: "<<name<<" attribute: "<<attribute<<endlog();
                            //check if there already exists an element with name
                            vector<recv_element_base*>::iterator it;
                            for(it=all_recv_elements.begin();it!=all_recv_elements.end();it++){
                                //log(Info)<<"iterator name: "<<(*it)->name<<endlog();

                                if((*it)->name==name){
                                    if(!(*it)->addAttribute(attribute)){
                                        log(Error)<<"Adding rec attribute "<<attribute <<"to element "<< (*it)->name <<"failed."<<endlog();
                                        return false;
                                    }
                                    break;
                                }
                            }

                            if(it==all_recv_elements.end()){

                                //New element
                                switch (type) {
                                    case BOOL:
                                        all_recv_elements.push_back(new recv_element_vector<bool>(name, type));
                                        break;
                                    case STRING:
                                        all_recv_elements.push_back(new recv_element_vector<string>(name, type));
                                        break;
                                    case LONG:
                                        all_recv_elements.push_back(new recv_element_vector<int>(name, type));
                                        break;
                                    case FLOAT:
                                        all_recv_elements.push_back(new recv_element_vector<float>(name, type));
                                        break;
                                    case DOUBLE:
                                        all_recv_elements.push_back(new recv_element_vector<double>(name, type));
                                        break;
                                    case NONE:
                                        log(Error)<<"NONE as element type"<<endlog();
                                        break;
                                    default:
                                        log(Error)<<"unknown element type"<<endlog();
                                        break;
                                }
                                if(!all_recv_elements.back()->addAttribute(attribute)){
                                    log(Error)<<"This should not occur rec"<<endlog();
                                    return false;
                                }
                                log(Info)<<"Adding new element "<< name << " with attribute "<<attribute<<endlog();
                                log(Debug)<<"element name "<< all_recv_elements.back()->name << " of type " << fromTypeToString(all_recv_elements.back()->type) <<  endlog();
                            }
                        }
                    }
                    
                }

                if(!prepareMessage()){
                    log(Error)<<"Failed to prepare the message for the robot"<<endlog();
                    return false;
                }
                
            } catch (ticpp::Exception& ex) {
                log(Error) << "XML Parsing exception: " << ex.what() << endlog();
                return false;
            }
            return true;
        }

        bool RSIComponent::prepareMessage(){
            recv_message = ticpp::Element("Sen");
            recv_message.SetAttribute<std::string>("Type", sentype);
            //iterate over all recv_elements:
            vector<recv_element_base*>::iterator it;
            for(it=all_recv_elements.begin();it!=all_recv_elements.end();it++){
                (*it)->ti_el = ticpp::Element((*it)->name);
                recv_message.LinkEndChild(&((*it)->ti_el));
            }
            ipoc_recv=ticpp::Element("IPOC");
            recv_message.LinkEndChild(&ipoc_recv);
            
            return true;
        }
        
        bool RSIComponent::configureHook() {
            
            //Try to connect
            connected = false;
            log(Info)<<"Using protocol "<<protocol<<endlog();
            
            if (protocol == "TCP"){
                serversock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
                protocol_is_udp=false;
            }
            else if (protocol == "UDP"){
                serversock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
                protocol_is_udp=true;
            }
            else {
                log(Error) << "Only TCP and UDP are supported as protocol"
                        << endlog();
                return false;
            }
            
            if (serversock < 0) {
                log(Error) << "Socket creation failed." << endlog();
                return false;
            }
            
            memset(&localserver, 0, sizeof(localserver));
            localserver.sin_family = AF_INET;
            localserver.sin_port = htons(port);
            localserver.sin_addr.s_addr = INADDR_ANY;
            
            if (bind(serversock, (struct sockaddr*) &localserver, sizeof(localserver)) < 0)
            {
                log(Error) << "Binding of port failed with errno " << errno
                        << endlog();
                return false;
            }
            
            if(!protocol_is_udp){
                if (listen(serversock, 2) < 0) {
                    log(Error) << "Cannot listen on socket" << endlog();
                    close(serversock);
                    return false;
                }
            }
            return true;
        }
        
        void RSIComponent::updateHook() {
            
            //log(Info)<<"Update hook!"<<endlog();
            if(protocol_is_udp){
                socklen_t size=sizeof(udpclient);
                if (recvfrom(serversock, buffer, 1024, 0, (struct sockaddr* )& udpclient , &size) < 0) {
                    log(Error) << "Failed to receive message" << endlog();
                }
                else{
                    message_received=true;
                }
            }
            else {
                if (!connected) {
                    socklen_t clientlen = sizeof(localclient);
                    // Wait for client connection
                    if ((clientsock = accept(serversock,
                            (struct sockaddr *) &localclient, &clientlen)) < 0) {
                        log(Warning) << "Failed to accept client connection"
                                << endlog();
                    }
                    else {
                        log(Info) << "Client connected: " << inet_ntoa(localclient.sin_addr) << endlog();
                        connected = true;
                    }
                }
                else {
                    if (recv(clientsock, buffer, 1024, 0) < 0) {
                        log(Error) << "Failed to receive message" << endlog();
                    }
                    else {
                        message_received=true;
                    }
                }
            }
            //
            // RECEIVE MESSAGE
            //
            if(message_received){
                message_received=false;
                //log(Info)<<"Message received!"<<endlog();
                
#ifndef NDEBUG
log(Debug)<<"Message received: "<<buffer<<endlog();
#endif

// Parse received message
bufferstring.assign(buffer);
#ifndef NDEBUG
log(Debug)<<"bufferstring: "<<bufferstring<<endlog();
#endif
// TESTCASE
try {
    doc_buffer.Clear();
    doc_buffer.Parse(bufferstring);
#ifndef NDEBUG
log(Debug)<<"Parsed message: "<<doc_buffer<<endlog();
#endif
//Get the Rob node:
rob = doc_buffer.FirstChildElement("Rob");

//Get the IPOC
ipoc_el = rob->FirstChildElement("IPOC");
ipoc_el->GetText<long long> (&ipoc);
#ifndef NDEBUG
log(Debug)<<"IPOC: "<<ipoc<<endlog();
#endif

} catch (ticpp::Exception ex) {
    log(Error) << ex.what() << endlog();
    return;
}

vector<element_base*>::const_iterator it;
for (it = all_send_elements.begin(); it
        != all_send_elements.end(); ++it) {
    //get the ticpp element for this send element

    try{
        ticpp::Element* el = rob->FirstChildElement((*it)->name);

        std::string name;
        RTT::base::OutputPortInterface* portp;
        name = (*it)->name;
        if( name == "RIst")
            portp = &send_RIst;
        else if( name == "RSol")
            portp = &send_RSol;
        else if( name == "AIPos")
            portp = &send_AIPos;
        else if( name == "ASPos")
            portp = &send_ASPos;
        else if( name == "MACur")
            portp = &send_MACur;
        else if( name == "Delay")
            portp = &send_Delay;

        (*it)->updatePort(el,portp);

    } catch(ticpp::Exception ex){
#ifndef NDEBUG
        log(Error)<<ex.what()<<endlog();
#endif
    }
}
//
// SEND MESSAGE
//

if (!only_send){
    vector<recv_element_base*>::const_iterator it;
    for (it = all_recv_elements.begin(); it
            != all_recv_elements.end(); ++it) {
        //get the ticpp element for this send element
        try{

            std::string name;
            RTT::base::InputPortInterface* portp;
            name = (*it)->name;
            if( name == "DEF_EStr")
                portp = &recv_EStr;
            else if( name == "AKorr")
                portp = &recv_AKorr;
            else if( name == "RKorr")
                portp = &recv_RKorr;

            (*it)->updateElement(portp);

        }catch(ticpp::Exception ex){
            log(Error)<<ex.what()<<endlog();
        }
    }
    ipoc_recv.SetText<long long>(ipoc);
    
    TiXmlPrinter printer;
    recv_message.Accept(&printer);
    
#ifndef NDEBUG
log(Debug)<<"Sending message:\n"<<printer.Str()<<endlog();
#endif

if(!protocol_is_udp){
    //if(send(clientsock, sendbuffer.str().c_str(), sendbuffer.str().size(),0)<0)
    if(send(clientsock, printer.CStr(), printer.Size(), 0)<0)
        log(Error)<<"Sending message failed"<<endlog();
#ifndef NDEBUG
    else
        log(Debug)<<"Message sent!"<<endlog();
#endif
}
else{
    size_t size=sizeof(udpclient);
    if(sendto(serversock, printer.CStr(), printer.Size(), 0, (struct sockaddr*) &udpclient, size)<0)
        log(Error)<<"Sending message failed"<<endlog();
#ifndef NDEBUG
    else
        log(Debug)<<"Message sent!"<<endlog();
#endif
}
}
            }
            this->trigger();
        }
        
        void RSIComponent::stopHook() {
            close(clientsock);
            close(serversock);
        }
        
        void RSIComponent::cleanupHook() {
        }
}
//Namespace KR5
ORO_CREATE_COMPONENT(KR5::RSIComponent)
