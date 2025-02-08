#include <WiFi.h>
#include <NetworkClientSecure.h>
#define PB_BUFFER_ONLY 1
#include <SnappyProto.h>
#include "proto/pb.h"
#include "proto/pb_decode.h"  // Don't need to include pb_encode.h
#include "gtfs-realtime.pb.h"
#include "boundingBox.h"

#include <Wire.h>
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"

//int ledPin = 10;

const long bufferLength = 32384;
uint8_t buffer[bufferLength];
const char *ssid = "JohnWiFi";
const char *password = "cleverpassword";


// Map all Bounding boxes to column/row
// Some of these overlap
// http://bboxfinder.com
static const Mapping stationMapping[] = {
  Mapping(BoundingBox(-84.511739, 39.097893, -84.510334, 39.100775), 7, 0),   // The Banks
  Mapping(BoundingBox(-84.510623, 39.097894, -84.509411, 39.098706), 7, 0),   // The Banks
  Mapping(BoundingBox(-84.509395, 39.097890, -84.508697, 39.099318), 7, 1),   // 4th & Main
  Mapping(BoundingBox(-84.509990, 39.099259, -84.508569, 39.100830), 7, 1),   // 4th & Main
  Mapping(BoundingBox(-84.510441, 39.100823, -84.508681, 39.103402), 7, 2),   // 6th & Main
  Mapping(BoundingBox(-84.511042, 39.103396, -84.509583, 39.105487), 7, 3),   // 8th & Main
  Mapping(BoundingBox(-84.511482, 39.105479, -84.510216, 39.107262), 7, 4),   // Court & Main
  Mapping(BoundingBox(-84.511760, 39.107242, -84.510494, 39.108609), 7, 5),   // Hanke Exchange
  Mapping(BoundingBox(-84.514958, 39.108148, -84.511734, 39.108808), 7, 6),   // 12th & Vine
  Mapping(BoundingBox(-84.511830, 39.108642, -84.511267, 39.108913), 7, 6),   // 12th & Vine
  Mapping(BoundingBox(-84.516309, 39.108035, -84.514952, 39.108451), 7, 7),   // 14th & Elm
  Mapping(BoundingBox(-84.517297, 39.107805, -84.516535, 39.108200), 7, 7),   // 14th & Elm
  Mapping(BoundingBox(-84.518938, 39.107540, -84.517227, 39.110017), 7, 7),   // 14th & Elm
  Mapping(BoundingBox(-84.520022, 39.109897, -84.518391, 39.113046), 7, 8),   // Liberty & Elm
  Mapping(BoundingBox(-84.520258, 39.112970, -84.518788, 39.115153), 7, 9),   // Findlay Market - Elm
  Mapping(BoundingBox(-84.520698, 39.115060, -84.519217, 39.117334), 7, 10),  // Brewery District
  Mapping(BoundingBox(-84.521057, 39.117329, -84.518434, 39.118287), 7, 11),  // Findlay Market - Race
  Mapping(BoundingBox(-84.519013, 39.115488, -84.517801, 39.117479), 7, 11),  // Findlay Market - Race
  Mapping(BoundingBox(-84.518434, 39.112501, -84.516985, 39.115558), 7, 12),  // Liberty & Race
  Mapping(BoundingBox(-84.517640, 39.108127, -84.516309, 39.112611), 7, 13),  // Washington Park
  Mapping(BoundingBox(-84.516444, 39.106691, -84.513981, 39.108012), 7, 14),  // Central Parkway
  Mapping(BoundingBox(-84.513992, 39.105184, -84.512056, 39.107365), 7, 15),  // Public Library
  Mapping(BoundingBox(-84.512587, 39.103953, -84.511836, 39.105245), 6, 0),   // Aronoff Center
  Mapping(BoundingBox(-84.512297, 39.100471, -84.510956, 39.104003), 6, 1),   // Fountain Square
};

constexpr size_t stationMappingLength = sizeof(stationMapping) / sizeof(stationMapping[0]);
static_assert(stationMappingLength == 24);

#ifndef _BV
#define _BV(bit) (1 << (bit))
#endif

Adafruit_8x16matrix matrix = Adafruit_8x16matrix();
uint16_t tempIncomingDisplayBuffer[8];

static const uint8_t PROGMEM
  test1[] = {
    B11111111,
    B00000000,
    B11111111,
    B00000000,
    B11111111,
    B00000000,
    B11111111,
    B00000000,
    B11111111,
    B00000000,
    B11111111,
    B00000000,
    B11111111,
    B00000000,
    B11111111,
    B00000000,
  },
  test2[] = {
    B00000000,
    B11111111,
    B00000000,
    B11111111,
    B00000000,
    B11111111,
    B00000000,
    B11111111,
    B00000000,
    B11111111,
    B00000000,
    B11111111,
    B00000000,
    B11111111,
    B00000000,
    B11111111,
  };



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(10);
  Serial.println("Starting matrix...");
  bool ret = Wire.setPins(21, 20);
  Serial.println("Success setting pins " + String(ret));
  ret = matrix.begin(0x70);
  Serial.println("Success beginning matrix " + String(ret));

  Serial.println();
  Serial.println();
  Serial.print("Waiting for WiFi...");
  // We start by connecting to a WiFi network
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }


  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  matrix.drawBitmap(0, 0, test1, 8, 16, LED_ON);
  matrix.writeDisplay();
  Serial.println("printed first");
  delay(500);

  matrix.clear();
  matrix.writeDisplay();
  Serial.println("cleared");
  delay(200);

  matrix.drawBitmap(0, 0, test2, 8, 16, LED_ON);
  matrix.writeDisplay();
  Serial.println("printed second");
  delay(500);

  matrix.clear();
  matrix.writeDisplay();
  Serial.println("cleared");
  delay(200);
}

uint8_t counter = 0;

bool decodeTripId(pb_istream_t *stream, const pb_field_t *field, void **arg) {
  size_t len = stream->bytes_left;
  // Read string into the allocated buffer
  if (!pb_read(stream, (pb_byte_t *)*arg, std::min(len, (size_t)19))) {
    return false;
  }

  return true;
}

bool decodeEntity(pb_istream_t *stream, const pb_field_t *field, void **arg) {
  transit_realtime_FeedEntity feedEntity = transit_realtime_FeedEntity_init_zero;
  char id[20];
  char tripId[20];
  memset(tripId, 0, 20);
  memset(id, 0, 20);
  feedEntity.id.funcs.decode = &decodeTripId;
  feedEntity.id.arg = &id;
  feedEntity.vehicle.trip.route_id.funcs.decode = &decodeTripId;
  feedEntity.vehicle.trip.route_id.arg = &tripId;
  if (!pb_decode(stream, transit_realtime_FeedEntity_fields, &feedEntity)) {
    Serial.println(String("Decoding faile Feed Entity!: ") + String(PB_GET_ERROR(stream)));
    return false;
  }
  // The streetcar is route 100. The protobuf looks like:
  /*
  entity {
  id: "1178B"
  vehicle {
    trip {
      trip_id: "1708906"
      start_date: "20241227"
      route_id: "100"
    }
  ... }
  */
  String stringTripId(tripId);
  if (stringTripId == "100" && feedEntity.has_vehicle && feedEntity.vehicle.has_position) {

    double lat = feedEntity.vehicle.position.latitude;
    double lon = feedEntity.vehicle.position.longitude;

    //Serial.println("Found Streetcar at position (lon, lat) " + String(lon) + ", " + String(lat));

    for (int i = 0; i < stationMappingLength; i++) {
      int col = stationMapping[i].m_col;
      int row = stationMapping[i].m_row;
      if (stationMapping[i].m_bb.contains(lon, lat)) {
        Serial.println("ID: " + String(id) + " Route: " + stringTripId + " within bb for col " + String(col) + " row: " + String(row));
        tempIncomingDisplayBuffer[7 - col] |= _BV(row);
      }
    }
  }

  return true;
}

void loop() {
  //matrix.clear();
  //matrix.writeDisplay();
  //Serial.println("cleared");
  //delay(200);


  const uint16_t port = 443;
  const char *host = "tmgtfsprd.sorttrpcloud.com";  // ip or dns

  Serial.print("Connecting to ");
  Serial.println(host);

  // Use NetworkClient class to create TCP connections
  NetworkClientSecure client;
  client.setInsecure();  //skip verification

  if (!client.connect(host, port)) {
    Serial.println("Connection failed.");
    Serial.println("Waiting 5 seconds before retrying...");
    delay(5000);
    return;
  }

  String footer = String(" HTTP/1.1\r\n") + "Host: " + String(host) + "\r\n" + "Connection: close\r\n\r\n";

  //client.print("GET /TMGTFSRealTimeWebService/vehicle/vehiclepositions.pb HTTP/1.1\r\nHost: tmgtfsprd.sorttrpcloud.com\r\nCustomUserAgent/1.0\r\nAccept: */*\r\nConnection: close\r\n\r\n");
  client.print("GET /TMGTFSRealTimeWebService/vehicle/vehiclepositions.pb" + footer);


  int maxloops = 0;

  //wait for the server's reply to become available
  while (!client.available() && maxloops < 1000) {
    maxloops++;
    delay(1);  //delay 1 msec
  }

  if (client.available() == 0) {
    Serial.println("client.available() timed out ");
    client.stop();
    return;
  }

  // 1) Search output for "Content-Length",
  // 2) Parse the content length
  // 3) Skip to the end of the header (look for \r\n\r\n)
  // 4) Read the remaining buffer
  // and then
  while (client.available() > 0) {
    // Each line should be terminated with "\r\n"
    String line = client.readStringUntil('\r');
    if (line.startsWith("\nContent-Length: ")) {
      long payloadLength = line.substring(17).toInt();
      Serial.println("ContentLength: " + String(payloadLength));
      while (client.available() > 0) {
        line = client.readStringUntil('\n');
        if (line == "\r") {
          // The end of the header is marked with "\r\n\r\n"
          // If we are here we've advanced to the final "\n", so the rest of the buffer should be the payload
          break;
        }
      }
      if (client.available() >= payloadLength && bufferLength >= payloadLength) {
        client.readBytes(buffer, payloadLength);
      } else {
        Serial.println("Specified payloadLength exceeds available buffer size: " + String(client.available()) + " Or maxBuffer length: " + String(bufferLength));
        break;
      }

      /* Allocate space for the decoded message. */
      transit_realtime_FeedMessage message = transit_realtime_FeedMessage_init_zero;
      message.entity.funcs.decode = &decodeEntity;
      message.entity.arg = nullptr;

      /* Create a stream that reads from the buffer. */
      pb_istream_t stream = pb_istream_from_buffer(buffer, payloadLength);

      for (int i = 0; i < 8; i++) {
        tempIncomingDisplayBuffer[i] = 0;
      }
      /* Now we are ready to decode the message. */
      int status = pb_decode(&stream, transit_realtime_FeedMessage_fields, &message);
      if (!status) {
        Serial.println("ERROR: pb_decode status: " + String(status));
        return;  // error condition
      }
      Serial.println("msg timestamp: " + String(message.header.timestamp));
      // Overlay updated positions on top of existing positions to give a sense of movement.
      for (int i = 0; i < 8; i++) {
        matrix.displaybuffer[i] |= tempIncomingDisplayBuffer[i];
      }
      matrix.writeDisplay();
      delay(2000);
      // Now we display only new positions.
      for (int i = 0; i < 8; i++) {
        matrix.displaybuffer[i] = tempIncomingDisplayBuffer[i];
      }
      matrix.writeDisplay();

      break;
    }
  }

  //Serial.println("Closing connection.");
  client.stop();

  Serial.println("Waiting 10 seconds before restarting...");
  delay(10000);
}
