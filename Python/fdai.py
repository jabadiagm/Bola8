import OpenGL.GLUT as glut
import OpenGL.GLU as glu
import OpenGL.GL as gl

import sys
import numpy as np
import ctypes as ctypes
import math as math
from PIL import Image as Image
from PIL import ImageOps as ImageOps
import time as time
import threading as threading
import platform as platform

#constants

#enum
LIGHT_DISABLE = 0;
LIGHT_POINT = 1;
LIGHT_DIR = 2;

CAMERA_FOVY = 18.6
CAMERA_ZFAR = 1000.0
CAMERA_ZNEAR = 0.1

#globals
g_demo = True
g_brightness = 40000
g_fpsStartTime = time.time()
g_frames = 0
g_startTime = time.time()
g_freq = 0.01
g_period = 1 / g_freq
g_lastTime = time.time()
g_renderBusy = False
g_dataAvailable = False
g_attitude = None
g_name = 'Apollo FDAI Demo'
g_viewRotX = 0.0
g_viewRotY = 0.0
g_cameraRadius = 450
g_multiSampling = True

#geometry
g_nullTexture = 0
g_colorMapTexture = 0
g_normalMapTexture = 0
g_disneyShaderProgram = 0
g_vertexBuffer = 0
g_vertexIndexBuffer = 0

g_vertexBufferEsfera = 0
g_esfera= []

g_vertexBufferEscalaRoll = 0
g_escalaRoll = []

g_vertexBufferEscalaError = 0
g_escalaError = []

g_vertexBufferEscalaExterior = 0
g_escalaExterior = []

g_vertexBufferMetalNegro = 0
g_metalNegro = []

g_vertexBufferTapa = 0
g_tapa = []

g_vertexBufferPlasticoNegro = 0
g_plasticoNegro = []

g_vertexBufferCruz = 0
g_cruz = []

g_vertexBufferFlecha = 0
g_flecha = []

g_vertexBufferFlechaRoll = 0
g_flechaRoll = []

g_vertexBufferTexturaFlechaRoll = 0
g_texturaFlechaRoll = []

g_vertexBufferAgujaRecta = 0
g_agujaRecta = []

g_vertexBufferAgujaCurva = 0
g_agujaCurva = []

g_vertexBufferOmega = 0
g_Omega = []

g_disableColorMapTexture = False
g_windowWidth = 640
g_windowHeight = 480

#materials
g_materialEsfera = None
g_materialEscalaRoll = None
g_materialEscalaError = None
g_materialEscalaExterior = None
g_materialMetalNegro = None
g_materialMetalGris = None
g_materialPlasticoNaranja = None
g_materialPlasticoBlanco = None
g_materialPlasticoNegro = None


class Attitude():
    roll = 0
    pitch = 0
    yaw = 0
    rateRoll = 0
    ratePitch = 0
    rateYaw = 0
    errorRoll = 0
    errorPitch = 0
    errorYaw = 0
    brightness = 0
    
    def __str__(self):
        return "roll="+str(self.roll)+"\n"+ \
            "pitch="+str(self.pitch)+"\n"+ \
            "yaw="+str(self.yaw)+"\n"+ \
            "rateRoll="+str(self.rateRoll) + "\n"+ \
            "ratePitch="+str(self.ratePitch) + "\n"+ \
            "rateYaw="+str(self.rateYaw) + "\n"+ \
            "errorRoll="+str(self.errorRoll) + "\n"+ \
            "errorPitch="+str(self.errorPitch) + "\n"+ \
            "errorYaw="+str(self.errorYaw) + "\n"+ \
            "brightness="+str(self.brightness)
    
def generateAttitude():
    global g_startTime
    
    currentTime = time.time()
    while (currentTime - g_startTime) >= g_period:
        g_startTime = g_startTime + g_period
    currentTime = currentTime - g_startTime
    
    result = Attitude()
    result.roll = currentTime / g_period * 360
    result.pitch = 0
    result.yaw = currentTime / g_period * 360
    result.rateRoll = 127 * math.sin(2*math.pi*g_freq*currentTime)
    result.ratePitch = 127 * math.sin(1*math.pi*g_freq*currentTime)
    result.rateYaw = -127 * math.sin(4*math.pi*g_freq*currentTime)
    result.errorRoll = 127 * math.sin(2*math.pi*g_freq*currentTime)
    result.errorPitch = 127 * math.sin(2*math.pi*g_freq*currentTime)
    result.errorYaw = 127 * math.sin(4*math.pi*g_freq*currentTime)
    result.brightness = g_brightness
    return result

def int2SignedByte(integer):
    if integer < 0:
        return 256 + integer
    else:
        return integer

def signedByte2Int(sbyte):
    if sbyte >= 128:
        return sbyte - 256
    else:
        return sbyte

def encodeAttitude(attitude):
    result = Attitude()
    result.roll = round(attitude.roll / 360 * 65536)
    result.pitch = round(attitude.pitch / 360 * 65536)
    result.yaw = round(attitude.yaw / 360 * 65536)
    result.rateRoll = int2SignedByte(round(attitude.rateRoll))
    result.ratePitch = int2SignedByte(round(attitude.ratePitch))
    result.rateYaw = int2SignedByte(round(attitude.rateYaw))
    result.errorRoll = int2SignedByte(round(attitude.errorRoll))
    result.errorPitch = int2SignedByte(round(attitude.errorPitch))
    result.errorYaw = int2SignedByte(round(attitude.errorYaw))
    result.brightness = int(g_brightness / 1000)
    return result

def decodeAttitude(message101, message102):
    result = Attitude()
    result.roll = (message101[0] + 256 * message101[1]) * 360 / 65536
    result.pitch = (message101[2] + 256 * message101[3]) * 360 / 65536
    result.yaw = (message101[4] + 256 * message101[5]) * 360 / 65536
    
    result.rateRoll = signedByte2Int(message102[0])
    result.ratePitch = signedByte2Int(message102[1])
    result.rateYaw = signedByte2Int(message102[2])
    
    result.errorRoll = signedByte2Int(message102[3])
    result.errorPitch = signedByte2Int(message102[4])
    result.errorYaw = signedByte2Int(message102[5])
    
    result.brightness = signedByte2Int(message102[6])
    
    return result
    

if platform.system()=='Linux':
    g_raspberry = True
else:
    g_raspberry = False
    
if g_raspberry:
    #can1 = server, can0=client
    import os as os
    import can as can
    import queue as queue
    
    global g_can0Queue
    global g_can1Queue
    
    os.system("sudo /sbin/ip link set can0 down")
    os.system("sudo /sbin/ip link set can1 down")
    os.system("sudo /sbin/ip link set can0 up type can bitrate 125000")
    os.system("sudo ifconfig can0 txqueuelen 4000")
    os.system("sudo /sbin/ip link set can1 up type can bitrate 125000")
    os.system("sudo ifconfig can1 txqueuelen 4000")
    
    g_can0 = can.interface.Bus(channel='can0', bustype='socketcan_native')
    g_can1 = can.interface.Bus(channel='can1', bustype='socketcan_native')
    
    g_can0Queue = queue.Queue()
    g_can1Queue = queue.Queue()
    
    class Can0Listener(object):
        def on_message_received(self, msg):
            g_can0Queue.put(msg)
    
        def __call__(self, msg):
            return self.on_message_received(msg)
    
        def stop(self):
            """
            Override to cleanup any open resources.
            """    
    def createCanMessage(id,data):
        return can.Message(arbitration_id=id,data=data,extended_id=False)
    
    def can0Task():
        global g_message101
        global g_dataAvailable
        global g_attitude
        
        while True:
            message = g_can0Queue.get()
            if message.arbitration_id == 0x101 or message.arbitration_id == 0x102:
                #print(message)
                if message.arbitration_id == 0x101:
                    g_message101 = message
                else:
                    newAttitude= decodeAttitude(g_message101.data, message.data)
                    #print(newAttitude)
                    if not g_renderBusy:
                        g_attitude = newAttitude
                        g_dataAvailable = True
                        while not g_renderBusy:
                            time.sleep(0.001)
                
                        g_dataAvailable = False

                
    def can1Send(attitude):
        encodedAttitude = encodeAttitude(attitude)

        message1=createCanMessage(0x101,[0x00,0x00, 0x00,0x00, 0x00,0x00])
        message1.data[0], message1.data[1] = encodedAttitude.roll.to_bytes(2,"little")
        message1.data[2], message1.data[3] = encodedAttitude.pitch.to_bytes(2,"little")
        message1.data[4], message1.data[5] = encodedAttitude.yaw.to_bytes(2,"little")
        g_can1.send(message1)
        
        message2=createCanMessage(0x102,[0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        message2.data[0] = encodedAttitude.rateRoll
        message2.data[1] = encodedAttitude.ratePitch
        message2.data[2] = encodedAttitude.rateYaw
        message2.data[3] = encodedAttitude.errorRoll
        message2.data[4] = encodedAttitude.errorPitch
        message2.data[5] = encodedAttitude.errorYaw
        message2.data[6] = encodedAttitude.brightness

        g_can1.send(message2)        
        
                
    def can1Task():
        while True:
            attitude = generateAttitude()
            can1Send(attitude)
            time.sleep(0.01)
            
    can0Listener = Can0Listener()
    can0Notifier = can.Notifier(g_can0, [can0Listener])
            
    t0=threading.Thread(target=can0Task)
    t0.start()
    t1=threading.Thread(target=can1Task)
    t1.start()
  
    


    

disney=([
    '''
#version 120

const int LIGHT_DISABLE = 0;
const int LIGHT_POINT = 1;
const int LIGHT_DIR = 2;


//uniform mat4 projectionMatrix;
//uniform mat4 modelViewMatrix;
uniform mat4 worldViewMatrix;
uniform int useColorTexture;
uniform int useNormalTexture;

//lights
uniform int light1Type = LIGHT_DISABLE;
uniform int light2Type = LIGHT_DISABLE;
uniform int light3Type = LIGHT_DISABLE;
uniform int light4Type = LIGHT_DISABLE;
uniform int light5Type = LIGHT_DISABLE;

uniform vec3 light1WorldPointDir = vec3(0);
uniform vec3 light2WorldPointDir = vec3(0);
uniform vec3 light3WorldPointDir = vec3(0);
uniform vec3 light4WorldPointDir = vec3(0);
uniform vec3 light5WorldPointDir = vec3(0);

//out vec4 worldSpaceVert;
varying vec3 eyeSpaceVert;
varying vec3 normalLocal;

varying vec3 light1LocalPointDir;
varying vec3 light2LocalPointDir;
varying vec3 light3LocalPointDir;
varying vec3 light4LocalPointDir;
varying vec3 light5LocalPointDir;

void main(void)
{
    // do the necessary transformations
    vec4 P = gl_ModelViewMatrix * gl_Vertex;
    vec4 light;
    //worldSpaceVert = vec4(gl_Vertex.xyz,1);
    eyeSpaceVert = vec3(gl_ModelViewMatrix * gl_Vertex);
    normalLocal = mat3(gl_ModelViewMatrix) * gl_Normal;
    
    gl_Position = gl_ProjectionMatrix * vec4(eyeSpaceVert,1);
    
    if (light1Type != LIGHT_DISABLE) {
        if (light1Type == LIGHT_POINT) {
            light =  worldViewMatrix * vec4(light1WorldPointDir,1);
            light1LocalPointDir = vec3(light - gl_ModelViewMatrix * gl_Vertex);
        } else {
            light =  worldViewMatrix * vec4(light1WorldPointDir,0);
            light1LocalPointDir = vec3(light);
        }
    }

    if (light2Type != LIGHT_DISABLE) {
        if (light2Type == LIGHT_POINT) {
            light =  worldViewMatrix * vec4(light2WorldPointDir,1);
            light2LocalPointDir = vec3(light - gl_ModelViewMatrix * gl_Vertex);
        } else {
            light =  worldViewMatrix * vec4(light2WorldPointDir,0);
            light2LocalPointDir = vec3(light);
        }
    }
            
    if (light3Type != LIGHT_DISABLE) {
        if (light3Type == LIGHT_POINT) {
            light =  worldViewMatrix * vec4(light3WorldPointDir,1);
            light3LocalPointDir = vec3(light - gl_ModelViewMatrix * gl_Vertex);
        } else {
            light =  worldViewMatrix * vec4(light3WorldPointDir,0);
            light3LocalPointDir = vec3(light);
        }
    }
            
    if (light4Type != LIGHT_DISABLE) {
        if (light4Type == LIGHT_POINT) {
            light =  worldViewMatrix * vec4(light4WorldPointDir,1);
            light4LocalPointDir = vec3(light - gl_ModelViewMatrix * gl_Vertex);
        } else {
            light =  worldViewMatrix * vec4(light4WorldPointDir,0);
            light4LocalPointDir = vec3(light);
        }
    }
            
    if (light5Type != LIGHT_DISABLE) {
        if (light5Type == LIGHT_POINT) {
            light =  worldViewMatrix * vec4(light5WorldPointDir,1);
            light5LocalPointDir = vec3(light - gl_ModelViewMatrix * gl_Vertex);
        } else {
            light =  worldViewMatrix * vec4(light5WorldPointDir,0);
            light5LocalPointDir = vec3(light);
        }
    }
            
    //if (useColorTexture != 0) {
        gl_TexCoord[0] = gl_MultiTexCoord0;
    //}

    if (useNormalTexture != 0) {
        vec3 n = normalize(gl_NormalMatrix * gl_Normal);
        vec3 t = normalize(gl_NormalMatrix * gl_MultiTexCoord1.xyz);
        vec3 b = cross(n, t) * gl_MultiTexCoord1.w;

        mat3 tbnMatrix = mat3(t.x, b.x, n.x,
                              t.y, b.y, n.y,
                              t.z, b.z, n.z);

        eyeSpaceVert = tbnMatrix * eyeSpaceVert;
        normalLocal = tbnMatrix * normalLocal;
        light1LocalPointDir = tbnMatrix * light1LocalPointDir;
        light2LocalPointDir = tbnMatrix * light2LocalPointDir;
        light3LocalPointDir = tbnMatrix * light3LocalPointDir;
        light4LocalPointDir = tbnMatrix * light4LocalPointDir;
        light5LocalPointDir = tbnMatrix * light5LocalPointDir;

        
    }

}

    ''',
], 
[
    '''
    
#version 120

//#define light5 //comment for raspberry

const int LIGHT_DISABLE = 0;
const int LIGHT_POINT = 1;
const int LIGHT_DIR = 2;

uniform int useColorTexture;
uniform int useNormalTexture;
uniform int useRoughnessTexture;
uniform int useMetalnessTexture;
uniform sampler2D colorMap;
uniform sampler2D normalMap;
uniform sampler2D roughnessMap;
uniform sampler2D metalnessMap;

uniform int light1Type = LIGHT_DISABLE;
uniform vec3 light1Color= vec3 ( 1.0, 1.0, 1.0);
uniform float light1Brightness = 1;

uniform int light2Type = LIGHT_DISABLE;
uniform vec3 light2Color= vec3 ( 1.0, 1.0, 1.0);
uniform float light2Brightness = 1;

uniform int light3Type = LIGHT_DISABLE;
uniform vec3 light3Color= vec3 ( 1.0, 1.0, 1.0);
uniform float light3Brightness = 1;

uniform int light4Type = LIGHT_DISABLE;
uniform vec3 light4Color= vec3 ( 1.0, 1.0, 1.0);
uniform float light4Brightness = 1;

uniform int light5Type = LIGHT_DISABLE;
uniform vec3 light5Color= vec3 ( 1.0, 1.0, 1.0);
uniform float light5Brightness = 1;


//uniform float gamma;
float gamma=2.2;
//uniform float exposure;
float exposure=1;
//uniform float useNDotL;
float useNDotL=1;

//in vec4 worldSpaceVert;
varying vec3 eyeSpaceVert;
varying vec3 light1LocalPointDir;
varying vec3 light2LocalPointDir;
varying vec3 light3LocalPointDir;
varying vec3 light4LocalPointDir;
varying vec3 light5LocalPointDir;
varying vec3 normalLocal;

//out vec4 fragColor;

uniform vec3 materialBaseColor = vec3 (.82, .67, .16);
uniform float materialMetallic = 0.0;
uniform float materialSubsurface = 0;
uniform float materialSpecular = 0.0;
uniform float materialRoughness = 0;
uniform float materialSpecularTint = 0;
uniform float materialAnisotropic = 0;
uniform float materialSheen = 0;
uniform float materialSheenTint = .5;
uniform float materialClearcoat = 0;
uniform float materialClearcoatGloss = 0;

vec3 baseColor;
float roughness;
float metalness;


//# Copyright Disney Enterprises, Inc.  All rights reserved.
//#
//# Licensed under the Apache License, Version 2.0 (the "License");
//# you may not use this file except in compliance with the License
//# and the following modification to it: Section 6 Trademarks.
//# deleted and replaced with:
//#
//# 6. Trademarks. This License does not grant permission to use the
//# trade names, trademarks, service marks, or product names of the
//# Licensor and its affiliates, except as required for reproducing
//# the content of the NOTICE file.
//#
//# You may obtain a copy of the License at
//# http://www.apache.org/licenses/LICENSE-2.0


const float PI = 3.14159265358979323846;

float sqr(float x) { return x*x; }

float SchlickFresnel(float u)
{
    float m = clamp(1-u, 0, 1);
    float m2 = m*m;
    return m2*m2*m; // pow(m,5)
}

float GTR1(float NdotH, float a)
{
    if (a >= 1) return 1/PI;
    float a2 = a*a;
    float t = 1 + (a2-1)*NdotH*NdotH;
    return (a2-1) / (PI*log(a2)*t);
}

float GTR2(float NdotH, float a)
{
    float a2 = a*a;
    float t = 1 + (a2-1)*NdotH*NdotH;
    return a2 / (PI * t*t);
}

float GTR2_aniso(float NdotH, float HdotX, float HdotY, float ax, float ay)
{
    return 1 / (PI * ax*ay * sqr( sqr(HdotX/ax) + sqr(HdotY/ay) + NdotH*NdotH ));
}

float smithG_GGX(float NdotV, float alphaG)
{
    float a = alphaG*alphaG;
    float b = NdotV*NdotV;
    return 1 / (NdotV + sqrt(a + b - a*b));
}

float smithG_GGX_aniso(float NdotV, float VdotX, float VdotY, float ax, float ay)
{
    return 1 / (NdotV + sqrt( sqr(VdotX*ax) + sqr(VdotY*ay) + sqr(NdotV) ));
}

vec3 mon2lin(vec3 x)
{
    return vec3(pow(x[0], 2.2), pow(x[1], 2.2), pow(x[2], 2.2));
}


vec3 BRDF( vec3 L, vec3 V, vec3 N, vec3 X, vec3 Y )
{
    float NdotL = dot(N,L);
    float NdotV = dot(N,V);
    if (NdotL < 0 || NdotV < 0) return vec3(0);

    vec3 H = normalize(L+V);
    float NdotH = dot(N,H);
    float LdotH = dot(L,H);

    vec3 Cdlin = mon2lin(baseColor);
    float Cdlum = .3*Cdlin[0] + .6*Cdlin[1]  + .1*Cdlin[2]; // luminance approx.

    vec3 Ctint = Cdlum > 0 ? Cdlin/Cdlum : vec3(1); // normalize lum. to isolate hue+sat
    vec3 Cspec0 = mix(materialSpecular*.08*mix(vec3(1), Ctint, materialSpecularTint), Cdlin, metalness);
    vec3 Csheen = mix(vec3(1), Ctint, materialSheenTint);

    // Diffuse fresnel - go from 1 at normal incidence to .5 at grazing
    // and mix in diffuse retro-reflection based on roughness
    float FL = SchlickFresnel(NdotL), FV = SchlickFresnel(NdotV);
    float Fd90 = 0.5 + 2 * LdotH*LdotH * roughness;
    float Fd = mix(1.0, Fd90, FL) * mix(1.0, Fd90, FV);

    // Based on Hanrahan-Krueger brdf approximation of isotropic bssrdf
    // 1.25 scale is used to (roughly) preserve albedo
    // Fss90 used to "flatten" retroreflection based on roughness
    float Fss90 = LdotH*LdotH*roughness;
    float Fss = mix(1.0, Fss90, FL) * mix(1.0, Fss90, FV);
    float ss = 1.25 * (Fss * (1 / (NdotL + NdotV) - .5) + .5);

    // specular
    float aspect = sqrt(1-materialAnisotropic*.9);
    float ax = max(.001, sqr(roughness)/aspect);
    float ay = max(.001, sqr(roughness)*aspect);
    float Ds = GTR2_aniso(NdotH, dot(H, X), dot(H, Y), ax, ay);
    float FH = SchlickFresnel(LdotH);
    vec3 Fs = mix(Cspec0, vec3(1), FH);
    float Gs;
    Gs  = smithG_GGX_aniso(NdotL, dot(L, X), dot(L, Y), ax, ay);
    Gs *= smithG_GGX_aniso(NdotV, dot(V, X), dot(V, Y), ax, ay);

    // sheen
    vec3 Fsheen = FH * materialSheen * Csheen;

    // clearcoat (ior = 1.5 -> F0 = 0.04)
    float Dr = GTR1(NdotH, mix(.1,.001,materialClearcoatGloss));
    float Fr = mix(.04, 1.0, FH);
    float Gr = smithG_GGX(NdotL, .25) * smithG_GGX(NdotV, .25);

    return ((1/PI) * mix(Fd, ss, materialSubsurface)*Cdlin + Fsheen)
        * (1-metalness)
        + Gs*Fs*Ds + .25*materialClearcoat*Gr*Fr*Dr;
}


vec3 computeWithDirectionalLight( vec3 surfPt, vec3 incidentVector, vec3 viewVec, vec3 normal, vec3 tangent, vec3 bitangent )
{
    // evaluate the BRDF
    vec3 b = max( BRDF( incidentVector, viewVec, normal, tangent, bitangent ), vec3(0.0) );

    // multiply in the cosine factor
    if (useNDotL != 0)
        b *= dot( normal, incidentVector );

    return b;
}


vec3 computeWithPointLight( vec3 surfPt, vec3 incidentVector, vec3 viewVec, vec3 normal, vec3 tangent, vec3 bitangent )
{
    // compute the point light vector
    vec3 toLight = (incidentVector); // * lightDistanceFromCenter);// - surfPt;
    float pointToLightDist = length( toLight );
    toLight /= pointToLightDist;


    // evaluate the BRDF
    vec3 b = max( BRDF( toLight, viewVec, normal, tangent, bitangent ), vec3(0.0) );

    // multiply in the cosine factor
    if (useNDotL != 0)
        b *= dot( normal, toLight );

    // multiply in the falloff
    b *= (1.0 / (pointToLightDist*pointToLightDist));

    return b;
}


void main(void)
{
    // orthogonal vectors
    //vec3 normal = normalize( worldSpaceVert.xyz );
    vec3 normal = normalize( normalLocal );
    vec3 tangent = normalize( cross( vec3(0,1,0), normal ) );
    vec3 bitangent = normalize( cross( normal, tangent ) );


    // calculate the viewing vector
    vec3 viewVec = -normalize(eyeSpaceVert.xyz);
    vec3 surfacePos = vec3(0); //normalize( worldSpaceVert.xyz );
    //vec3 viewVec = vec3(0,0,1); // ortho mode
    
    if (useColorTexture != 0) {
        baseColor = vec3(texture2D(colorMap, gl_TexCoord[0].st));
    } else {
        baseColor = materialBaseColor;
    }
        
    if (useNormalTexture != 0) {
        normal = normalize(texture2D(normalMap, gl_TexCoord[0].st).xyz * 2.0 - 1.0);
    } 

    if (useRoughnessTexture != 0) {
        roughness = vec3(texture2D(roughnessMap, gl_TexCoord[0].st)).y;
    } else {
        roughness = materialRoughness;
    }    
        
    if (useMetalnessTexture != 0) {
        metalness = vec3(texture2D(metalnessMap, gl_TexCoord[0].st)).y;
    } else {
        metalness = materialMetallic;
    }         

    vec3 b = vec3(0);
    if (light1Type != LIGHT_DISABLE) {
        vec3 light1b = vec3(0);
        if (light1Type == LIGHT_POINT) {
            light1b += light1Color * computeWithPointLight(  surfacePos, light1LocalPointDir, viewVec, normal, tangent, bitangent );
        } else {
            light1b += light1Color * computeWithDirectionalLight( surfacePos, normalize(light1LocalPointDir), viewVec, normal, tangent, bitangent );
        } 
        light1b *= light1Brightness;
        b += light1b;
    }
        
    if (light2Type != 0) {
        vec3 light2b = vec3(0);
        if (light2Type == LIGHT_POINT) {
            light2b += light2Color * computeWithPointLight(  surfacePos, light2LocalPointDir, viewVec, normal, tangent, bitangent );
        } else {
            light2b += light2Color * computeWithDirectionalLight( surfacePos, normalize(light2LocalPointDir), viewVec, normal, tangent, bitangent );
        } 
        light2b *= light2Brightness;
        b += light2b;
    } 
            
    if (light3Type != 0) {
        vec3 light3b = vec3(0);
        if (light3Type == LIGHT_POINT) {
            light3b += light3Color * computeWithPointLight(  surfacePos, light3LocalPointDir, viewVec, normal, tangent, bitangent );
        } else {
            light3b += light3Color * computeWithDirectionalLight( surfacePos, normalize(light3LocalPointDir), viewVec, normal, tangent, bitangent );
        } 
        light3b *= light3Brightness;
        b += light3b;
    }
            
    if (light4Type != 0) {
        vec3 light4b = vec3(0);
        if (light4Type == LIGHT_POINT) {
            light4b += light4Color * computeWithPointLight(  surfacePos, light4LocalPointDir, viewVec, normal, tangent, bitangent );
        } else {
            light4b += light4Color * computeWithDirectionalLight( surfacePos, normalize(light4LocalPointDir), viewVec, normal, tangent, bitangent );
        } 
        light4b *= light4Brightness;
        b += light4b;
    } 
            
    #ifdef light5
        if (light5Type != 0) {
            vec3 light5b = vec3(0);
            if (light5Type == LIGHT_POINT) {
                light5b += light5Color * computeWithPointLight(  surfacePos, light5LocalPointDir, viewVec, normal, tangent, bitangent );
            } else {
                light5b += light5Color * computeWithDirectionalLight( surfacePos, normalize(light5LocalPointDir), viewVec, normal, tangent, bitangent );
            } 
            light5b *= light5Brightness;
            b += light5b;
        }
    #endif
        
        
    //vec3 b = computeWithAreaLight( surfacePos, incidentVector, viewVec, normal, tangent, bitangent );

    // exposure
    b *= pow( 2.0, exposure );

    // gamma
    b = pow( b, vec3( 1.0 / gamma ) );

    gl_FragColor = vec4( clamp( b, vec3(0.0), vec3(1.0) ), 1.0 );
    //gl_FragColor = vec4( 1,1,1, 1.0 );
}
    ''',
])






def main():
    
    global g_colorMapTexture
    global g_normalMapTexture
    global g_roughnessMapTexture
    global g_metalnessMapTexture
    global g_nullTexture
    global g_disneyShaderProgram
    global g_attitude

    
    glut.glutInit(sys.argv)
    
    if not g_multiSampling:
        glut.glutInitDisplayMode(glut.GLUT_DOUBLE | glut.GLUT_RGB | glut.GLUT_DEPTH)
    else:
        glut.glutSetOption(glut.GLUT_MULTISAMPLE, 8)
        glut.glutInitDisplayMode(glut.GLUT_DOUBLE | glut.GLUT_RGB | glut.GLUT_DEPTH | glut.GLUT_MULTISAMPLE)
    
    glut.glutInitWindowPosition(0, 0)
    glut.glutInitWindowSize(g_windowWidth, g_windowHeight)
    glut.glutCreateWindow(g_name)
    glut.glutFullScreen()
    
    #glut.glutDisplayFunc(display)
    glut.glutDisplayFunc(RenderFrame)
    glut.glutKeyboardFunc(key)
    glut.glutReshapeFunc(reshape)
    glut.glutSpecialFunc(special)
    glut.glutVisibilityFunc(visible)   
    
    GL_MAX_VARYING_FLOATS = gl.glGetIntegerv(gl.GL_MAX_VARYING_FLOATS) 
    print("GL_MAX_VARYING_FLOATS=",GL_MAX_VARYING_FLOATS)
    
    #InitApp

    # Load the GLSL normal mapping shaders
    g_disneyShaderProgram=LoadShaderProgram(disney)
    print("g_disneyShaderProgram=",g_disneyShaderProgram)

    InicializarMateriales()
    InicializarGeometria()
    #print(IndicesEsfera)
    
    #Setup initial rendering states
    
    gl.glEnable(gl.GL_TEXTURE_2D)
    gl.glEnable(gl.GL_DEPTH_TEST)
    gl.glEnable(gl.GL_CULL_FACE)
    gl.glEnable(gl.GL_LIGHTING)
    gl.glEnable(gl.GL_LIGHT0)   
    
    g_attitude = Attitude()
    
 
    
    #while(True):
    #    RenderFrame()
    #    UpdateFrameRate()

    
    glut.glutMainLoop()
    
    #print(g_cube)
    
    return

def InicializarMateriales():
    global g_materialEsfera
    global g_materialMetalNegro
    global g_materialEscalaRoll
    global g_materialEscalaError
    global g_materialEscalaExterior
    global g_materialMetalGris
    global g_materialPlasticoNaranja
    global g_materialPlasticoBlanco
    global g_materialPlasticoNegro
    
    g_materialEsfera = Material()
    g_materialEsfera.roughness = 1
    g_materialEsfera.colorMapTexture = LoadTexture("Bola8_1800_Color.jpg")
    g_materialEsfera.normalMapTexture = LoadTexture("Bola8_1800_Normal.jpg")
    g_materialEsfera.roughnessMapTexture = LoadTexture("Bola8_1K_Roughness.jpg")
    
    g_materialMetalNegro = Material()
    #g_materialMetalNegro.baseColor = [0.05, 0.05, 0.05]
    g_materialMetalNegro.specular = 0.0
    #g_materialMetalNegro.roughness = 0
    g_materialMetalNegro.metallic = 0.0
    g_materialMetalNegro.colorMapTexture = LoadTexture("Metal_Negro_1K_Color.jpg")
    g_materialMetalNegro.normalMapTexture = LoadTexture("Metal_Negro_1K_Normal.jpg", True)
    g_materialMetalNegro.roughnessMapTexture = LoadTexture("Metal_Negro_1K_Roughness.jpg")
    
    g_materialMetalGris = Material()
    g_materialMetalGris.specular = 0
    g_materialMetalGris.roughness = 0
    g_materialMetalGris.metallic = 0.0
    g_materialMetalGris.colorMapTexture = LoadTexture("Metal_Gris_1K_Color.jpg")
    g_materialMetalGris.normalMapTexture = LoadTexture("Metal_Gris_1K_Normal.jpg")
    g_materialMetalGris.roughnessMapTexture = LoadTexture("Metal_Gris_1K_Roughness.jpg")

    g_materialEscalaRoll = Material()
    g_materialEscalaRoll.baseColor = g_materialMetalNegro.baseColor
    g_materialEscalaRoll.roughness = 1
    g_materialEscalaRoll.specular = 0
    g_materialEscalaRoll.metallic = 0.0
    g_materialEscalaRoll.colorMapTexture = LoadTexture("Escala_Roll_1K_Color.jpg")
    g_materialEscalaRoll.normalMapTexture = g_materialMetalNegro.normalMapTexture
    g_materialEscalaRoll.roughnessMapTexture = g_materialMetalNegro.roughnessMapTexture
    
    g_materialEscalaError = Material()
    g_materialEscalaError.baseColor = g_materialMetalNegro.baseColor
    g_materialEscalaError.roughness = 1
    g_materialEscalaError.specular = 0
    g_materialEscalaError.metallic = 0.0
    g_materialEscalaError.colorMapTexture = LoadTexture("Escala_Error_1K_Color.jpg", True)
    g_materialEscalaError.normalMapTexture = g_materialMetalNegro.normalMapTexture
    g_materialEscalaError.roughnessMapTexture = g_materialMetalNegro.roughnessMapTexture
    
    g_materialEscalaExterior = Material()
    #g_materialEscalaExterior.baseColor = g_materialMetalNegro.baseColor
    g_materialEscalaExterior.roughness = 1
    g_materialEscalaExterior.specular = 0
    g_materialEscalaExterior.metallic = 0
    g_materialEscalaExterior.colorMapTexture = LoadTexture("Escala_Exterior_1K_Color.jpg",True)
    g_materialEscalaExterior.normalMapTexture = g_materialMetalNegro.normalMapTexture
    g_materialEscalaExterior.roughnessMapTexture = g_materialMetalNegro.roughnessMapTexture

    g_materialPlasticoNaranja = Material()
    g_materialPlasticoNaranja.baseColor = [1, 0.7, .0]
    g_materialPlasticoNaranja.specular = 0
    g_materialPlasticoNaranja.metallic = 0.5
    g_materialPlasticoNaranja.roughness = 0.5
    
    g_materialPlasticoBlanco = Material()
    g_materialPlasticoBlanco.baseColor = [0.8, 0.8, 0.8]
    g_materialPlasticoBlanco.specular = 0
    g_materialPlasticoBlanco.metallic = 0
    g_materialPlasticoBlanco.roughness = 1.0
    
    g_materialPlasticoNegro = Material()
    g_materialPlasticoNegro.baseColor = [0.1, 0.1, 0.1]
    g_materialPlasticoNegro.specular = 0
    g_materialPlasticoNegro.metallic = 0
    g_materialPlasticoNegro.roughness = 1.0    
    

def InicializarGeometria():
    global g_vertexBufferEsfera
    global g_esfera    
    global g_vertexBufferEscalaRoll 
    global g_escalaRoll
    global g_vertexBufferEscalaError
    global g_escalaError
    global g_vertexBufferEscalaExterior
    global g_escalaExterior
    global g_vertexBufferMetalNegro
    global g_metalNegro
    global g_vertexBufferTapa
    global g_tapa
    global g_vertexBufferPlasticoNegro
    global g_plasticoNegro
    global g_vertexBufferCruz
    global g_cruz
    global g_vertexBufferFlecha
    global g_flecha
    global g_vertexBufferFlechaRoll
    global g_flechaRoll
    global g_vertexBufferTexturaFlechaRoll
    global g_texturaFlechaRoll
    global g_vertexBufferAgujaRecta
    global g_agujaRecta
    global g_vertexBufferAgujaCurva
    global g_agujaCurva
    global g_vertexBufferOmega
    global g_omega
    
    print("Generando geometría de esfera")
    g_esfera = np.load("esfera.npy")
    print("Cargando geometría de esfera en opengl")
    g_vertexBufferEsfera = gl.glGenBuffers(1)
    gl.glBindBuffer(gl.GL_ARRAY_BUFFER, g_vertexBufferEsfera)
    gl.glBufferData(gl.GL_ARRAY_BUFFER, len(g_esfera) * 12 * 4, g_esfera, gl.GL_STATIC_DRAW)
    gl.glBindBuffer(gl.GL_ARRAY_BUFFER, 0)
    print("Geometría de esfera OK")
    
    print("Generando geometría de escala de roll")
    g_escalaRoll = np.load("escala_roll.npy")
    print("Cargando geometría de escala de roll en opengl")
    g_vertexBufferEscalaRoll = gl.glGenBuffers(1)
    gl.glBindBuffer(gl.GL_ARRAY_BUFFER, g_vertexBufferEscalaRoll)
    gl.glBufferData(gl.GL_ARRAY_BUFFER, len(g_escalaRoll) * 12 * 4, g_escalaRoll, gl.GL_STATIC_DRAW)
    gl.glBindBuffer(gl.GL_ARRAY_BUFFER, 0)
    print("Geometría OK")    
    
    print("Generando geometría de escala de error")
    g_escalaError = np.load("escala_error.npy")
    print("Cargando geometría de escala de error en opengl")
    g_vertexBufferEscalaError = gl.glGenBuffers(1)
    gl.glBindBuffer(gl.GL_ARRAY_BUFFER, g_vertexBufferEscalaError)
    gl.glBufferData(gl.GL_ARRAY_BUFFER, len(g_escalaError) * 12 * 4, g_escalaError, gl.GL_STATIC_DRAW)
    gl.glBindBuffer(gl.GL_ARRAY_BUFFER, 0)
    print("Geometría de escala de error OK")  

    print("Generando geometría de escala exterior")
    g_escalaExterior = np.load("escala_exterior.npy")
    print("Cargando geometría de escala exterior en opengl")
    g_vertexBufferEscalaExterior = gl.glGenBuffers(1)
    gl.glBindBuffer(gl.GL_ARRAY_BUFFER, g_vertexBufferEscalaExterior)
    gl.glBufferData(gl.GL_ARRAY_BUFFER, len(g_escalaExterior) * 12 * 4, g_escalaExterior, gl.GL_STATIC_DRAW)
    gl.glBindBuffer(gl.GL_ARRAY_BUFFER, 0)
    print("Geometría de escala exterior OK")
    
    print("Generando geometría de metal negro: carcasas interior y exterior")
    g_metalNegro = np.load("metal_negro.npy")
    print("Cargando geometría de metal negro en opengl")
    g_vertexBufferMetalNegro = gl.glGenBuffers(1)
    gl.glBindBuffer(gl.GL_ARRAY_BUFFER, g_vertexBufferMetalNegro)
    gl.glBufferData(gl.GL_ARRAY_BUFFER, len(g_metalNegro) * 12 * 4, g_metalNegro, gl.GL_STATIC_DRAW)
    gl.glBindBuffer(gl.GL_ARRAY_BUFFER, 0)
    print("Geometría de metal negro OK")     
    
    print("Generando geometría de tapa")
    g_tapa = np.load("tapa.npy")
    print("Cargando geometría de tapa en opengl")
    g_vertexBufferTapa = gl.glGenBuffers(1)
    gl.glBindBuffer(gl.GL_ARRAY_BUFFER, g_vertexBufferTapa)
    gl.glBufferData(gl.GL_ARRAY_BUFFER, len(g_tapa) * 12 * 4, g_tapa, gl.GL_STATIC_DRAW)
    gl.glBindBuffer(gl.GL_ARRAY_BUFFER, 0)
    print("Geometría de tapa OK")

    print("Generando geometría de plástico negro: alambres y textura de cruz")
    g_plasticoNegro = np.load("plastico_negro.npy")
    print("Cargando geometría de plástico negro en opengl")
    g_vertexBufferPlasticoNegro = gl.glGenBuffers(1)
    gl.glBindBuffer(gl.GL_ARRAY_BUFFER, g_vertexBufferPlasticoNegro)
    gl.glBufferData(gl.GL_ARRAY_BUFFER, len(g_plasticoNegro) * 12 * 4, g_plasticoNegro, gl.GL_STATIC_DRAW)
    gl.glBindBuffer(gl.GL_ARRAY_BUFFER, 0)
    print("Geometría de plástico negro OK")

    print("Generando geometría de cruz")
    g_cruz = np.load("cruz.npy")
    print("Cargando geometría de cruz en opengl")
    g_vertexBufferCruz = gl.glGenBuffers(1)
    gl.glBindBuffer(gl.GL_ARRAY_BUFFER, g_vertexBufferCruz)
    gl.glBufferData(gl.GL_ARRAY_BUFFER, len(g_cruz) * 12 * 4, g_cruz, gl.GL_STATIC_DRAW)
    gl.glBindBuffer(gl.GL_ARRAY_BUFFER, 0)
    print("Geometría de cruz OK")
    

    print("Generando geometría de flecha")
    g_flecha = np.load("flecha.npy")
    print("Cargando geometría de flecha en opengl")
    g_vertexBufferFlecha = gl.glGenBuffers(1)
    gl.glBindBuffer(gl.GL_ARRAY_BUFFER, g_vertexBufferFlecha)
    gl.glBufferData(gl.GL_ARRAY_BUFFER, len(g_flecha) * 12 * 4, g_flecha, gl.GL_STATIC_DRAW)
    gl.glBindBuffer(gl.GL_ARRAY_BUFFER, 0)
    print("Geometría de flecha OK")

    print("Generando geometría de flecha de roll")
    g_flechaRoll = np.load("flecha_roll.npy")
    print("Cargando geometría de flecha de roll en opengl")
    g_vertexBufferFlechaRoll = gl.glGenBuffers(1)
    gl.glBindBuffer(gl.GL_ARRAY_BUFFER, g_vertexBufferFlechaRoll)
    gl.glBufferData(gl.GL_ARRAY_BUFFER, len(g_flechaRoll) * 12 * 4, g_flechaRoll, gl.GL_STATIC_DRAW)
    gl.glBindBuffer(gl.GL_ARRAY_BUFFER, 0)
    print("Geometría de flecha de roll OK")    

    print("Generando geometría de textura de flecha de roll")
    g_texturaFlechaRoll = np.load("textura_flecha_roll.npy")
    print("Cargando geometría de textura de flecha de roll en opengl")
    g_vertexBufferTexturaFlechaRoll = gl.glGenBuffers(1)
    gl.glBindBuffer(gl.GL_ARRAY_BUFFER, g_vertexBufferTexturaFlechaRoll)
    gl.glBufferData(gl.GL_ARRAY_BUFFER, len(g_texturaFlechaRoll) * 12 * 4, g_texturaFlechaRoll, gl.GL_STATIC_DRAW)
    gl.glBindBuffer(gl.GL_ARRAY_BUFFER, 0)
    print("Geometría de textura de flecha de roll OK")    

    print("Generando geometría de aguja recta")
    g_agujaRecta = np.load("aguja_recta.npy")
    print("Cargando geometría de aguja recta en opengl")
    g_vertexBufferAgujaRecta = gl.glGenBuffers(1)
    gl.glBindBuffer(gl.GL_ARRAY_BUFFER, g_vertexBufferAgujaRecta)
    gl.glBufferData(gl.GL_ARRAY_BUFFER, len(g_agujaRecta) * 12 * 4, g_agujaRecta, gl.GL_STATIC_DRAW)
    gl.glBindBuffer(gl.GL_ARRAY_BUFFER, 0)
    print("Geometría de aguja recta OK")

    print("Generando geometría de aguja curva")
    g_agujaCurva = np.load("aguja_curva.npy")
    print("Cargando geometría de aguja curva en opengl")
    g_vertexBufferAgujaCurva = gl.glGenBuffers(1)
    gl.glBindBuffer(gl.GL_ARRAY_BUFFER, g_vertexBufferAgujaCurva)
    gl.glBufferData(gl.GL_ARRAY_BUFFER, len(g_agujaCurva) * 12 * 4, g_agujaCurva, gl.GL_STATIC_DRAW)
    gl.glBindBuffer(gl.GL_ARRAY_BUFFER, 0)
    print("Geometría de aguja curva OK")    

    print("Generando geometría de omega")
    g_omega = np.load("omega.npy")
    print("Cargando geometría de omega en opengl")
    g_vertexBufferOmega = gl.glGenBuffers(1)
    gl.glBindBuffer(gl.GL_ARRAY_BUFFER, g_vertexBufferOmega)
    gl.glBufferData(gl.GL_ARRAY_BUFFER, len(g_omega) * 12 * 4, g_omega, gl.GL_STATIC_DRAW)
    gl.glBindBuffer(gl.GL_ARRAY_BUFFER, 0)
    print("Geometría de omega OK") 


def CalcularNormal(punto1, punto2, punto3):
    vector12 = punto2 - punto1
    vector13 = punto3 - punto1
    normal = np.cross (vector12, vector13)
    return normal / np.linalg.norm(normal)

class Material():
    baseColor = [.82, .67, .16]
    metallic = 0.0
    subsurface = 0
    specular = 0.0
    roughness = 0.0
    specularTint = 0
    anisotropic = 0
    sheen = 0
    sheenTint = .5
    clearcoat = 0
    clearcoatGloss = 0
    
    colorMapTexture = 0
    normalMapTexture = 0
    roughnessMapTexture = 0
    metalnessMapTexture = 0
    


    
    
def normalize(vector):
    #divides a numpy vector by its module
    return vector/np.linalg.norm(vector)

def CreateNullTexture(width, height):
    # Create an empty white texture. This texture is applied to models
    # that don't have any texture maps. This trick allows the same shader to
    # be used to draw the model with and without textures applied.    
    
    pitch = ((width * 32 + 31) & bit_not(31,8)) >> 3 # align to 4-byte boundaries
    pixels = np.full ( pitch * height, 255,np.uint8)
    texture = gl.glGenTextures(1)
    gl.glBindTexture(gl.GL_TEXTURE_2D, texture)

    gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MAG_FILTER, gl.GL_NEAREST)
    gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MIN_FILTER, gl.GL_NEAREST)
    gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_WRAP_S, gl.GL_REPEAT)
    gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_WRAP_T, gl.GL_REPEAT)
    
    gl.glTexImage2D(gl.GL_TEXTURE_2D, 0, gl.GL_RGBA8, width, height, 0, gl.GL_BGRA, gl.GL_UNSIGNED_BYTE, pixels)
    
    return texture

def bit_not(n, numbits=8):
    return (1 << numbits) - 1 - n

def LoadTexture(fileName, flip=False):
    print("Textura=",fileName)
    return LoadTexture2(fileName, gl.GL_LINEAR, gl.GL_LINEAR_MIPMAP_LINEAR,
        gl.GL_REPEAT, gl.GL_REPEAT, flip)

def LoadTexture2(fileName, magFilter, minFilter, wrapS, wrapT, flip):
    img = Image.open(fileName)
    if flip:
        img = ImageOps.flip(img)
    #img_data = np.array(list(img.getdata()), np.int8)
    img_data =np.asarray( img, dtype="int8")
    id = gl.glGenTextures(1)
    gl.glBindTexture(gl.GL_TEXTURE_2D, id)
    """
    gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MAG_FILTER, magFilter)
    gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MIN_FILTER, minFilter)
    gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_WRAP_S, wrapS)
    gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_WRAP_T, wrapT)
    gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_GENERATE_MIPMAP, gl.GL_TRUE)
    """
    
    gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MIN_FILTER, gl.GL_LINEAR)
    gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MAG_FILTER, gl.GL_LINEAR)
    gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_WRAP_S, gl.GL_REPEAT)
    gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_WRAP_T, gl.GL_REPEAT)
    
    gl.glTexImage2D(gl.GL_TEXTURE_2D, 0, gl.GL_RGB, img.size[0],img.size[1], 0, gl.GL_RGB, gl.GL_UNSIGNED_BYTE, img_data)
    gl.glBindTexture(gl.GL_TEXTURE_2D, 0)
    print("Textura OK")
    return id


def LoadShaderProgram(shaderProgram):
    vertShader = CompileShader(gl.GL_VERTEX_SHADER, shaderProgram[0])
    fragShader = CompileShader(gl.GL_FRAGMENT_SHADER, shaderProgram[1])
    program = LinkShaders(vertShader, fragShader)
    return program

def CompileShader(type, source):
    # Compiles the shader given it's source code. Returns the shader object.
    # 'type' is either GL_VERTEX_SHADER or GL_FRAGMENT_SHADER.
    shader = gl.glCreateShader(type)
    if shader:
        gl.glShaderSource(shader, source)
        gl.glCompileShader(shader)
        compiled=gl.glGetShaderiv(shader, gl.GL_COMPILE_STATUS)
        if not compiled:
            raise Exception("Compile error") 
    return shader
    
def LinkShaders(vertShader, fragShader):
    # Links the compiled vertex and/or fragment shaders into an executable
    # shader program. Returns the executable shader object. If the shaders
    # failed to link into an executable shader program, then a std::string
    # object is thrown containing the info log.    
    program = gl.glCreateProgram()
    if program:
        gl.glAttachShader(program, vertShader)
        gl.glAttachShader(program, fragShader)
        gl.glLinkProgram(program)
        linked=gl.glGetProgramiv(program, gl.GL_LINK_STATUS)
        if not linked:
            raise Exception("Linker error")   
        # Mark the two attached shaders for deletion. These two shaders aren't
        # deleted right now because both are already attached to a shader
        # program. When the shader program is deleted these two shaders will
        # be automatically detached and deleted.            
        gl.glDeleteShader(vertShader)
        gl.glDeleteShader(fragShader)
    return program

def cosd(degrees):
    return math.cos(degrees * math.pi / 180)

def sind(degrees):
    return math.sin(degrees * math.pi / 180)

def key(k, x, y):
    global light1Type
    global light2Type
    global g_disableColorMapTexture
    global g_brightness
    if k == b'z':
        g_attitude.roll += 5.0
    elif k == b'Z':
        g_attitude.roll -= 5.0
    elif k == b'x':
        g_attitude.pitch += 5.0
    elif k == b'X':
        g_attitude.pitch -= 5.0
    elif k == b'y':
        g_attitude.yaw += 5.0
    elif k == b'Y':
        g_attitude.yaw -= 5.0
    elif k == b't':
        g_disableColorMapTexture = not g_disableColorMapTexture
    elif k == b'1':
        light1Type = (light1Type + 1) %3
    elif k == b'2':
        light2Type = (light2Type + 1) %3
    elif k == b'+':
        g_brightness += 5000
    elif k == b'-':
        g_brightness -= 5000
        if g_brightness < 0:
            g_brightness = 0
    elif ord(k) == 27: # Escape
        sys.exit(0)
    else:
        return
    print("roll=",g_attitude.roll,"pitch=",g_attitude.pitch,"yaw=",g_attitude.yaw,"brillo=",g_brightness)
    glut.glutPostRedisplay()
    
# change view angle
def special(k, x, y):
    global g_viewRotX
    global g_viewRotY
    
    if k == glut.GLUT_KEY_UP:
        g_viewRotX -= 5.0
    elif k == glut.GLUT_KEY_DOWN:
        g_viewRotX += 5.0
    elif k == glut.GLUT_KEY_LEFT:
        g_viewRotY -= 5.0
    elif k == glut.GLUT_KEY_RIGHT:
        g_viewRotY += 5.0
    else:
        return
    print("g_viewRotX=",g_viewRotX,"g_viewRotY=",g_viewRotY)
    glut.glutPostRedisplay()

# new window size or exposure
def reshape(width, height):
    global g_windowWidth
    global g_windowHeight
    g_windowWidth = width
    g_windowHeight = height

def visible(vis):
    if vis == glut.GLUT_VISIBLE:
        glut.glutIdleFunc(idle)
    else:
        glut.glutIdleFunc(None)  

def idle():
    global g_dataAvailable
    global g_renderBusy
    global g_attitude
    
    g_renderBusy = False
    if not g_raspberry and g_demo:
        g_attitude = generateAttitude()
        g_dataAvailable = True
    while not g_dataAvailable and g_demo:
        time.sleep(0.01)
    g_renderBusy = True
    #UpdateFrameRate()
    glut.glutPostRedisplay()
    #time.sleep(0.1)
    

def UpdateFrameRate():
    global g_fpsStartTime
    global g_frames
    if time.time() - g_fpsStartTime > 1.0:
        g_fpsStartTime = time.time()
        print("FPS = ",g_frames)
        g_frames = 0
    
def CargarMaterialOpengl (material):
    
    #cargar texturas
    gl.glActiveTexture(gl.GL_TEXTURE3)
    gl.glEnable(gl.GL_TEXTURE_2D)
    gl.glBindTexture(gl.GL_TEXTURE_2D, material.metalnessMapTexture)    
    
    gl.glActiveTexture(gl.GL_TEXTURE2)
    gl.glEnable(gl.GL_TEXTURE_2D)
    gl.glBindTexture(gl.GL_TEXTURE_2D, material.roughnessMapTexture)

    gl.glActiveTexture(gl.GL_TEXTURE1)
    gl.glEnable(gl.GL_TEXTURE_2D)
    gl.glBindTexture(gl.GL_TEXTURE_2D, material.normalMapTexture)
    
    gl.glActiveTexture(gl.GL_TEXTURE0)
    gl.glEnable(gl.GL_TEXTURE_2D)
    gl.glBindTexture(gl.GL_TEXTURE_2D, material.colorMapTexture)    
    
    gl.glUniform1i(gl.glGetUniformLocation(g_disneyShaderProgram, "colorMap"), 0)
    gl.glUniform1i(gl.glGetUniformLocation(g_disneyShaderProgram, "useColorTexture"), int(material.colorMapTexture != 0) )
        
    gl.glUniform1i(gl.glGetUniformLocation(g_disneyShaderProgram, "normalMap"), 1)
    gl.glUniform1i(gl.glGetUniformLocation(g_disneyShaderProgram, "useNormalTexture"), int(material.normalMapTexture != 0) )

    gl.glUniform1i(gl.glGetUniformLocation(g_disneyShaderProgram, "roughnessMap"), 2)
    gl.glUniform1i(gl.glGetUniformLocation(g_disneyShaderProgram, "useRoughnessTexture"), int(material.roughnessMapTexture != 0) )

    gl.glUniform1i(gl.glGetUniformLocation(g_disneyShaderProgram, "metalnessMap"), 3)
    gl.glUniform1i(gl.glGetUniformLocation(g_disneyShaderProgram, "useMetalnessTexture"), int(material.metalnessMapTexture != 0) )
    
    gl.glUniform3fv(gl.glGetUniformLocation(g_disneyShaderProgram, "materialBaseColor"),1, material.baseColor)
    gl.glUniform1f(gl.glGetUniformLocation(g_disneyShaderProgram, "materialMetallic"), material.metallic)
    gl.glUniform1f(gl.glGetUniformLocation(g_disneyShaderProgram, "materialSubsurface"), material.subsurface)
    gl.glUniform1f(gl.glGetUniformLocation(g_disneyShaderProgram, "materialSpecular"), material.specular)
    gl.glUniform1f(gl.glGetUniformLocation(g_disneyShaderProgram, "materialRoughness"), material.roughness)
    gl.glUniform1f(gl.glGetUniformLocation(g_disneyShaderProgram, "materialSpecularTint"), material.specularTint)
    gl.glUniform1f(gl.glGetUniformLocation(g_disneyShaderProgram, "materialAnisotropic"), material.anisotropic)
    gl.glUniform1f(gl.glGetUniformLocation(g_disneyShaderProgram, "materialSheen"), material.sheen)
    gl.glUniform1f(gl.glGetUniformLocation(g_disneyShaderProgram, "materialSheenTint"), material.sheenTint)
    gl.glUniform1f(gl.glGetUniformLocation(g_disneyShaderProgram, "materialClearcoat"), material.clearcoat)
    gl.glUniform1f(gl.glGetUniformLocation(g_disneyShaderProgram, "materialClearcoatGloss"), material.clearcoatGloss)
    
    
def PreRender(vertexBuffer, data):
    gl.glBindBuffer(gl.GL_ARRAY_BUFFER, vertexBuffer)
    
    gl.glEnableClientState(gl.GL_VERTEX_ARRAY)
    gl.glVertexPointer(3, gl.GL_FLOAT, data.strides[0], ctypes.c_void_p(0))
    
    gl.glClientActiveTexture(gl.GL_TEXTURE0);
    gl.glEnableClientState(gl.GL_TEXTURE_COORD_ARRAY);
    gl.glTexCoordPointer(2, gl.GL_FLOAT, data.strides[0], ctypes.c_void_p(12))
    
    gl.glEnableClientState(gl.GL_NORMAL_ARRAY)
    gl.glNormalPointer(gl.GL_FLOAT, data.strides[0], ctypes.c_void_p(20))
    
    gl.glClientActiveTexture(gl.GL_TEXTURE1)
    gl.glEnableClientState(gl.GL_TEXTURE_COORD_ARRAY)
    gl.glTexCoordPointer(4, gl.GL_FLOAT, data.strides[0], ctypes.c_void_p(32))    
    
def PostRender():
    #quitar texturas
    gl.glDisableClientState(gl.GL_TEXTURE_COORD_ARRAY)
    gl.glDisableClientState(gl.GL_NORMAL_ARRAY)
    gl.glClientActiveTexture(gl.GL_TEXTURE0)
    gl.glDisableClientState(gl.GL_TEXTURE_COORD_ARRAY)
    gl.glDisableClientState(gl.GL_VERTEX_ARRAY)

    gl.glBindBuffer(gl.GL_ARRAY_BUFFER, 0)

    gl.glActiveTexture(gl.GL_TEXTURE3)
    gl.glBindTexture(gl.GL_TEXTURE_2D, 0)
    gl.glDisable(gl.GL_TEXTURE_2D)

    gl.glActiveTexture(gl.GL_TEXTURE2)
    gl.glBindTexture(gl.GL_TEXTURE_2D, 0)
    gl.glDisable(gl.GL_TEXTURE_2D)

    gl.glActiveTexture(gl.GL_TEXTURE1)
    gl.glBindTexture(gl.GL_TEXTURE_2D, 0)
    gl.glDisable(gl.GL_TEXTURE_2D)

    gl.glActiveTexture(gl.GL_TEXTURE0)
    gl.glBindTexture(gl.GL_TEXTURE_2D, 0)
    gl.glDisable(gl.GL_TEXTURE_2D)    
    
def RenderEsfera():
    
    #material
    CargarMaterialOpengl (g_materialEsfera)
    
    #vbo
    PreRender (g_vertexBufferEsfera, g_esfera)
    
    #render
    gl.glPushMatrix()
    
    gl.glRotatef(90, 0, 1, 0)
    #roll
    gl.glRotatef(-g_attitude.roll, 1, 0, 0)
    #yaw
    gl.glRotatef(-g_attitude.yaw, 0, 1, 0)
    #pitch
    gl.glRotatef(g_attitude.pitch, 0, 0, 1)    
    
    gl.glDrawArrays(gl.GL_TRIANGLES, 0, len(g_esfera))
    gl.glPopMatrix()
    
def RenderEscalaRoll():
    
    #material
    CargarMaterialOpengl (g_materialEscalaRoll)
    
    #vbo
    PreRender (g_vertexBufferEscalaRoll, g_escalaRoll )
    
    #render
    gl.glPushMatrix()
    gl.glTranslatef(0.0, 0.0, 86)
    gl.glDrawArrays(gl.GL_TRIANGLES, 0, len(g_escalaRoll))
    gl.glPopMatrix()
    
def RenderEscalaError():
    
    #material
    CargarMaterialOpengl (g_materialEscalaError)
    
    #vbo
    PreRender (g_vertexBufferEscalaError, g_escalaError )
    
    #render
    gl.glPushMatrix()
    gl.glTranslatef(0.0, 0.0, 86)
    gl.glDrawArrays(gl.GL_TRIANGLES, 0, len(g_escalaError))
    gl.glPopMatrix()
    
def RenderEscalaExterior():
    
    #material
    CargarMaterialOpengl (g_materialEscalaExterior)
    
    #vbo
    PreRender (g_vertexBufferEscalaExterior, g_escalaExterior )
    
    #render
    gl.glPushMatrix()
    gl.glTranslatef(0.0, 0.0, 86)
    gl.glDrawArrays(gl.GL_TRIANGLES, 0, len(g_escalaExterior))
    gl.glRotatef( -90, 0, 0, 1)
    gl.glDrawArrays(gl.GL_TRIANGLES, 0, len(g_escalaExterior))
    gl.glRotatef( -90, 0, 0, 1)
    gl.glDrawArrays(gl.GL_TRIANGLES, 0, len(g_escalaExterior))
    gl.glPopMatrix()
    
def RenderEscalaExterior1():
    
    #material
    CargarMaterialOpengl (g_materialEscalaExterior)
    
    #vbo
    PreRender (g_vertexBufferEscalaExterior, g_escalaExterior )
    
    #render
    gl.glPushMatrix()
    gl.glTranslatef(0.0, 0.0, 86)
    gl.glDrawArrays(gl.GL_TRIANGLES, 0, len(g_escalaExterior))
    gl.glPopMatrix()
    
def RenderEscalaExterior2():
    
    #render
    gl.glPushMatrix()
    gl.glTranslatef(0.0, 0.0, 86)
    gl.glRotatef( -90, 0, 0, 1)
    gl.glDrawArrays(gl.GL_TRIANGLES, 0, len(g_escalaExterior))
    gl.glPopMatrix()  
    
def RenderEscalaExterior3():
    
    #render
    gl.glPushMatrix()
    gl.glTranslatef(0.0, 0.0, 86)
    gl.glRotatef( -180, 0, 0, 1)
    gl.glDrawArrays(gl.GL_TRIANGLES, 0, len(g_escalaExterior))
    gl.glPopMatrix()     
    
def RenderMetalNegro():
    
    #material
    CargarMaterialOpengl (g_materialMetalNegro)
    
    #vbo
    PreRender (g_vertexBufferMetalNegro, g_metalNegro )
    
    #render
    gl.glPushMatrix()
    gl.glTranslatef(0.0, 0.0, 86)
    gl.glDrawArrays(gl.GL_TRIANGLES, 0, len(g_metalNegro))
    gl.glPopMatrix()
    
def RenderTapa():
    
    #material
    CargarMaterialOpengl (g_materialMetalGris)
    
    #vbo
    PreRender (g_vertexBufferTapa, g_tapa )
    
    #render
    gl.glPushMatrix()
    gl.glTranslatef(0.0, 0.0, 86)
    gl.glDrawArrays(gl.GL_TRIANGLES, 0, len(g_tapa))
    gl.glPopMatrix()
    
def RenderPlasticoNegro():
    
    #material
    CargarMaterialOpengl (g_materialPlasticoNegro)
    
    #vbo
    PreRender (g_vertexBufferPlasticoNegro, g_plasticoNegro )
    
    #render
    gl.glPushMatrix()
    gl.glTranslatef(0.0, 0.0, 86)
    gl.glDrawArrays(gl.GL_TRIANGLES, 0, len(g_plasticoNegro))
    gl.glPopMatrix()
    
def RenderCruz():
    
    #material
    CargarMaterialOpengl (g_materialPlasticoNaranja)
    
    #vbo
    PreRender (g_vertexBufferCruz, g_cruz )
    
    #render
    gl.glPushMatrix()
    gl.glTranslatef(0.0, 0.0, 86)
    gl.glDrawArrays(gl.GL_TRIANGLES, 0, len(g_cruz))
    gl.glPopMatrix()
    
def RenderFlechas():
    
    #material
    CargarMaterialOpengl (g_materialPlasticoNegro)
    
    #vbo
    PreRender (g_vertexBufferFlecha, g_flecha )
    
    #render
    gl.glPushMatrix()
    
    gl.glTranslatef(0.0, 0.0, -67)
    gl.glRotatef( -g_attitude.rateRoll/10.2, 0, 1, 0)
    gl.glTranslatef(0.0, 0.0, 86+67)
    gl.glDrawArrays(gl.GL_TRIANGLES, 0, len(g_flecha))
    gl.glPopMatrix()
    
    gl.glPushMatrix()
    gl.glRotatef( -90, 0, 0, 1)
    gl.glTranslatef(0.0, 0.0, -67)
    gl.glRotatef( -g_attitude.ratePitch/10.2, 0, 1, 0)
    gl.glTranslatef(0.0, 0.0, 86+67)
    gl.glDrawArrays(gl.GL_TRIANGLES, 0, len(g_flecha))
    gl.glPopMatrix()
    
    gl.glPushMatrix()
    gl.glRotatef( -180, 0, 0, 1)

    gl.glTranslatef(0.0, 0.0, -67)
    gl.glRotatef( g_attitude.rateYaw/10.2, 0, 1, 0)
    gl.glTranslatef(0.0, 0.0, 86+67)

    gl.glDrawArrays(gl.GL_TRIANGLES, 0, len(g_flecha))
    gl.glPopMatrix()
    
def RenderFlechaRoll():
    
    #material
    CargarMaterialOpengl (g_materialPlasticoBlanco)
    
    #vbo
    PreRender (g_vertexBufferFlechaRoll, g_flechaRoll )
    
    #render
    gl.glPushMatrix()
    gl.glTranslatef(0.0, 0.0, 86)
    #roll
    gl.glRotatef(g_attitude.roll, 0, 0, 1)
    gl.glDrawArrays(gl.GL_TRIANGLES, 0, len(g_flechaRoll))
    gl.glPopMatrix()
    
def RenderTexturaFlechaRoll():
    
    #material
    CargarMaterialOpengl (g_materialPlasticoNegro)
    
    #vbo
    PreRender (g_vertexBufferTexturaFlechaRoll, g_texturaFlechaRoll )
    
    #render
    gl.glPushMatrix()
    gl.glTranslatef(0.0, 0.0, 86.1)
    #roll
    gl.glRotatef(g_attitude.roll, 0, 0, 1)
    gl.glDrawArrays(gl.GL_TRIANGLES, 0, len(g_texturaFlechaRoll))
    gl.glPopMatrix()
    
def RenderAgujasRectas():
    
    #material
    CargarMaterialOpengl (g_materialPlasticoNaranja)
    
    #vbo
    PreRender (g_vertexBufferAgujaRecta, g_agujaRecta )
    
    #render
    gl.glPushMatrix()
    gl.glTranslatef(0.0, 0.0, 86)
    gl.glTranslatef(-g_attitude.errorRoll/4.9, 0.0, 0.0)
    gl.glDrawArrays(gl.GL_TRIANGLES, 0, len(g_agujaRecta))
    gl.glTranslatef(g_attitude.errorRoll/4.9, 0.0, 0.0)
    gl.glRotatef( 180, 0, 0, 1)
    gl.glTranslatef(g_attitude.errorYaw/4.9, 0.0, 0.0)
    gl.glDrawArrays(gl.GL_TRIANGLES, 0, len(g_agujaRecta))
    gl.glTranslatef(-g_attitude.errorYaw/4.9, 0.0, 0.0)
    gl.glPopMatrix()
    
def RenderAgujaCurva():
    
    #material
    CargarMaterialOpengl (g_materialPlasticoNaranja)
    
    #vbo
    PreRender (g_vertexBufferAgujaCurva, g_agujaCurva )
    
    #render
    gl.glPushMatrix()
    gl.glTranslatef(0.0, 0.0, 86)
    gl.glTranslatef(0.0, g_attitude.errorPitch/4.9, 0.0)
    gl.glDrawArrays(gl.GL_TRIANGLES, 0, len(g_agujaCurva))
    gl.glPopMatrix()
    
def RenderOmega():
    
    #material
    CargarMaterialOpengl (g_materialPlasticoBlanco)
    
    #vbo
    PreRender (g_vertexBufferOmega, g_omega )
    
    #render
    gl.glPushMatrix()
    gl.glTranslatef(0.0, 0.0, 86)
    gl.glDrawArrays(gl.GL_TRIANGLES, 0, len(g_omega))
    gl.glPopMatrix()
    
def RenderFrame():
    global g_frames
    
    # Setup view port
    gl.glViewport(0, 0, g_windowWidth, g_windowHeight)
    gl.glClearColor(0.0, 0.0, 0.0, 0.0)
    gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT) 
    
   
    # Setup perspective projection matrix
    gl.glMatrixMode(gl.GL_PROJECTION)
    gl.glLoadIdentity()
    glu.gluPerspective(CAMERA_FOVY, g_windowWidth / g_windowHeight, CAMERA_ZNEAR, CAMERA_ZFAR)

    # Setup view matrix
    gl.glMatrixMode(gl.GL_MODELVIEW)
    gl.glLoadIdentity()
    glu.gluLookAt(g_cameraRadius * sind(g_viewRotX) + 9, g_viewRotY, g_cameraRadius * cosd(g_viewRotX),  9, 0, 0,  1.0, 0.0, 0.0)
    
    gl_worldview_matrix = (gl.GLfloat * 16)()
    gl.glGetFloatv(gl.GL_MODELVIEW_MATRIX, gl_worldview_matrix)
    #print ("gl_worldview_matrix=",list(gl_worldview_matrix))    
    
    
    # Setup lighting and shaders
    gl.glPushAttrib(gl.GL_LIGHTING_BIT)
    
    gl.glUseProgram(g_disneyShaderProgram)
    gl.glUniformMatrix4fv(gl.glGetUniformLocation(g_disneyShaderProgram, "worldViewMatrix"), 1, gl.GL_FALSE, gl_worldview_matrix)
    
       
    
    #luces esfera
    if g_brightness > 0:
        gl.glUniform1i(gl.glGetUniformLocation(g_disneyShaderProgram, "light1Type"), LIGHT_POINT)
        gl.glUniform3fv(gl.glGetUniformLocation(g_disneyShaderProgram, "light1Color"),1, [1.0, 1.0, 1.0])
        gl.glUniform3fv(gl.glGetUniformLocation(g_disneyShaderProgram, "light1WorldPointDir"),1, [0.0, 0, 200])
        gl.glUniform1f(gl.glGetUniformLocation(g_disneyShaderProgram, "light1Brightness"), g_brightness)
    else:
        gl.glUniform1i(gl.glGetUniformLocation(g_disneyShaderProgram, "light1Type"), LIGHT_DISABLE)
    
    gl.glUniform1i(gl.glGetUniformLocation(g_disneyShaderProgram, "light2Type"), LIGHT_POINT)
    gl.glUniform3fv(gl.glGetUniformLocation(g_disneyShaderProgram, "light2Color"),1, [0.5, 0.5, 1.0])
    gl.glUniform3fv(gl.glGetUniformLocation(g_disneyShaderProgram, "light2WorldPointDir"),1, [40, 40, 80])
    gl.glUniform1f(gl.glGetUniformLocation(g_disneyShaderProgram, "light2Brightness"), 2000)    
    gl.glUniform1i(gl.glGetUniformLocation(g_disneyShaderProgram, "light3Type"), LIGHT_POINT)
    gl.glUniform3fv(gl.glGetUniformLocation(g_disneyShaderProgram, "light3Color"),1, [0.5, 0.5, 1.0])
    gl.glUniform3fv(gl.glGetUniformLocation(g_disneyShaderProgram, "light3WorldPointDir"),1, [-40, 40, 80])
    gl.glUniform1f(gl.glGetUniformLocation(g_disneyShaderProgram, "light3Brightness"), 2000)     
    gl.glUniform1i(gl.glGetUniformLocation(g_disneyShaderProgram, "light4Type"), LIGHT_POINT)
    gl.glUniform3fv(gl.glGetUniformLocation(g_disneyShaderProgram, "light4Color"),1, [0.5, 0.5, 1.0])
    gl.glUniform3fv(gl.glGetUniformLocation(g_disneyShaderProgram, "light4WorldPointDir"),1, [0, -56, 80])
    gl.glUniform1f(gl.glGetUniformLocation(g_disneyShaderProgram, "light4Brightness"), 2000)       
    gl.glUniform1i(gl.glGetUniformLocation(g_disneyShaderProgram, "light5Type"), LIGHT_DISABLE)

    RenderEsfera()
    
    #luces escalas roll y error
    if g_brightness > 0:
        gl.glUniform1i(gl.glGetUniformLocation(g_disneyShaderProgram, "light1Type"), LIGHT_POINT)
        gl.glUniform3fv(gl.glGetUniformLocation(g_disneyShaderProgram, "light1Color"),1, [1.0, 1.0, 1.0])
        gl.glUniform3fv(gl.glGetUniformLocation(g_disneyShaderProgram, "light1WorldPointDir"),1, [20, 20, 200])
        gl.glUniform1f(gl.glGetUniformLocation(g_disneyShaderProgram, "light1Brightness"), g_brightness)
        
        
    gl.glUniform3fv(gl.glGetUniformLocation(g_disneyShaderProgram, "light2WorldPointDir"),1, [0, 78, 120])
    gl.glUniform3fv(gl.glGetUniformLocation(g_disneyShaderProgram, "light3WorldPointDir"),1, [78, 0, 120])
    gl.glUniform3fv(gl.glGetUniformLocation(g_disneyShaderProgram, "light4WorldPointDir"),1, [0, -78, 120])
        
    if g_brightness < 20000:
        gl.glUniform1i(gl.glGetUniformLocation(g_disneyShaderProgram, "light2Type"), LIGHT_POINT)
        gl.glUniform1i(gl.glGetUniformLocation(g_disneyShaderProgram, "light3Type"), LIGHT_POINT)
        gl.glUniform1i(gl.glGetUniformLocation(g_disneyShaderProgram, "light4Type"), LIGHT_POINT)
    else:
        gl.glUniform1i(gl.glGetUniformLocation(g_disneyShaderProgram, "light2Type"), LIGHT_DISABLE)
        gl.glUniform1i(gl.glGetUniformLocation(g_disneyShaderProgram, "light3Type"), LIGHT_DISABLE)
        gl.glUniform1i(gl.glGetUniformLocation(g_disneyShaderProgram, "light4Type"), LIGHT_DISABLE)

    RenderEscalaRoll()
    
    RenderEscalaError()
    
    gl.glUniform1i(gl.glGetUniformLocation(g_disneyShaderProgram, "light3Type"), LIGHT_DISABLE)
    gl.glUniform1i(gl.glGetUniformLocation(g_disneyShaderProgram, "light4Type"), LIGHT_DISABLE)    
    RenderEscalaExterior1()

    gl.glUniform1i(gl.glGetUniformLocation(g_disneyShaderProgram, "light2Type"), LIGHT_DISABLE)
    if g_brightness < 20000:
        gl.glUniform1i(gl.glGetUniformLocation(g_disneyShaderProgram, "light3Type"), LIGHT_POINT)

    RenderEscalaExterior2()

    gl.glUniform1i(gl.glGetUniformLocation(g_disneyShaderProgram, "light3Type"), LIGHT_DISABLE)
    if g_brightness < 20000:
        gl.glUniform1i(gl.glGetUniformLocation(g_disneyShaderProgram, "light4Type"), LIGHT_POINT)  
    RenderEscalaExterior3()
    
    gl.glUniform1i(gl.glGetUniformLocation(g_disneyShaderProgram, "light2Type"), LIGHT_POINT)
    gl.glUniform3fv(gl.glGetUniformLocation(g_disneyShaderProgram, "light2WorldPointDir"),1, [20, 20, 200])
    gl.glUniform1f(gl.glGetUniformLocation(g_disneyShaderProgram, "light2Brightness"), 12000)
    #gl.glUniform1i(gl.glGetUniformLocation(g_disneyShaderProgram, "light3Type"), LIGHT_DISABLE)
    gl.glUniform1i(gl.glGetUniformLocation(g_disneyShaderProgram, "light4Type"), LIGHT_DISABLE)

    
    RenderFlechaRoll()
    RenderTexturaFlechaRoll()
    RenderCruz()
    RenderPlasticoNegro()
    RenderAgujasRectas()
    RenderAgujaCurva()
    RenderOmega()
    
    gl.glUniform1i(gl.glGetUniformLocation(g_disneyShaderProgram, "light2Type"), LIGHT_DISABLE)
    #gl.glUniform1i(gl.glGetUniformLocation(g_disneyShaderProgram, "light3Type"), LIGHT_DISABLE)
    #gl.glUniform1i(gl.glGetUniformLocation(g_disneyShaderProgram, "light4Type"), LIGHT_DISABLE)
    #gl.glUniform1i(gl.glGetUniformLocation(g_disneyShaderProgram, "light5Type"), LIGHT_DISABLE)
    
    
    
    #RenderMetalNegro()
    #RenderTapa()

    RenderFlechas()
    
    #quitar texturas
    PostRender()
    

    gl.glUseProgram(0)
    
    gl.glPopAttrib()
    
    glut.glutSwapBuffers()
    
    g_frames = g_frames + 1
    
    return


if __name__ == '__main__': 
    main()
