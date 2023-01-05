/*
 * SDLogfile.cpp
 *
 *  Created on: Jan 3, 2023
 *      Author: cre
 */

#include "SDLogfile.hpp"

#include <SPI.h>
#include <SD.h>

// For file access on micro SD card
File myFile;
const int chipSelect = 53;

SDLogfile::SDLogfile ()
{
}

void SDLogfile::open ()
{
    Serial.print("Initializing SD card...");

    if (!SD.begin(chipSelect))
    {
        Serial.println("Could not initialize SD car...!");
    }
    Serial.println("SD card initialization done.");
    Serial.println("Creating logfile.txt...");
    myFile = SD.open("logfile.txt", FILE_WRITE);
}

void SDLogfile::close ()
{
    if (myFile)
    {
        myFile.close();
    }
}

void SDLogfile::print (String text)
{
    if (myFile)
    {
        myFile.print(text);
    }
}

void SDLogfile::print (char *text)
{
    if (myFile)
    {
        myFile.print(text);
    }
}
