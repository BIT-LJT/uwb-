#pragma once

#define LOG_NEWLINE            Serial.println("\n\n")
#define LOG(fmt, ...)          Serial.printf(fmt, ##__VA_ARGS__)
#define LOG_DEBUG(fmt, ...)    Serial.print("DEBUG: ");Serial.printf(fmt, ##__VA_ARGS__)
#define LOG_INFO(fmt, ...)     Serial.print("INFO: ") ;Serial.printf(fmt, ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...)    Serial.print("ERROR: ");Serial.printf(fmt, ##__VA_ARGS__)

#define LOGLN(fmt, ...)        Serial.println(fmt, ##__VA_ARGS__)
#define LOGLN_DEBUG(fmt, ...)  Serial.print("DEBUG: ");Serial.println(fmt, ##__VA_ARGS__)
#define LOGLN_INFO(fmt, ...)   Serial.print("INFO: ") ;Serial.println(fmt, ##__VA_ARGS__)
#define LOGLN_ERROR(fmt, ...)  Serial.print("ERROR: ");Serial.println(fmt, ##__VA_ARGS__)
