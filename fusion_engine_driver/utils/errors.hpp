// Copyright 2025 AI Racing Tech
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef FUSION_ENGINE_DRIVER_UTILS_ERRORS_HPP_
#define FUSION_ENGINE_DRIVER_UTILS_ERRORS_HPP_

#include <string_view>
#include <unordered_map>

enum class Errors : uint16_t {
  noError = 0,

  // General
  unknownError = 10000,
  invalidCommand,
  invalidArguments,
  notImplemented,

  // Parsing
  invalidPacket,
  crcMismatch,
  incompletePayload,
  unsupportedMessageType,
  missingHeader,

  // I/O & Stream
  readError,
  writeError,
  seekError,
  connectionLost,
  timeout,

  // GNSS / SBF specific
  unknownSbfBlock,
  invalidSbfLength,
  unsupportedRevision,
  invalidCrc,
  alignmentError,

  // Fusion Engine specific
  decodingFailed,
  conversionError,
  dataOutOfRange,
  inconsistentTimestamp,
};

struct StatusResult {
    Errors    error;

    StatusResult(Errors anError = Errors::noError) : error(anError) {}
    StatusResult& operator=(const StatusResult& aCopy) {
        error = aCopy.error;
        return *this;
    }
    operator bool() {return Errors::noError==error;}
    bool operator==(Errors anError) {return anError==error;}
    
  };

  inline std::string_view getError(Errors err) {
  static const std::unordered_map<Errors, std::string_view> kErrorMessages = {
    {Errors::noError, "No error"},
    {Errors::unknownError, "Unknown error"},
    {Errors::invalidCommand, "Invalid command"},
    {Errors::invalidArguments, "Invalid arguments"},
    {Errors::notImplemented, "Not implemented"},
    {Errors::invalidPacket, "Invalid packet"},
    {Errors::crcMismatch, "CRC mismatch"},
    {Errors::incompletePayload, "Incomplete payload"},
    {Errors::unsupportedMessageType, "Unsupported message type"},
    {Errors::missingHeader, "Missing message header"},
    {Errors::readError, "Read error"},
    {Errors::writeError, "Write error"},
    {Errors::seekError, "Seek error"},
    {Errors::connectionLost, "Connection lost"},
    {Errors::timeout, "Timeout"},
    {Errors::unknownSbfBlock, "Unknown SBF block"},
    {Errors::invalidSbfLength, "Invalid SBF length"},
    {Errors::unsupportedRevision, "Unsupported SBF revision"},
    {Errors::invalidCrc, "Invalid SBF CRC"},
    {Errors::alignmentError, "Alignment error"},
    {Errors::decodingFailed, "Message decoding failed"},
    {Errors::conversionError, "Data conversion error"},
    {Errors::dataOutOfRange, "Data out of range"},
    {Errors::inconsistentTimestamp, "Inconsistent timestamp"},
  };

  auto it = kErrorMessages.find(err);
  return (it != kErrorMessages.end()) ? it->second : "Unrecognized error";
}

  #endif // FUSION_ENGINE_DRIVER_UTILS_ERRORS_HPP_