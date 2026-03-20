#pragma once

#include <map>
#include <string>
#include <memory>
#include <vector>
#include <cstdint>

namespace auto_apms_behavior_codec
{
  // Class representation of a telemetry message, work in progress.
  class TelemetryMessage{
    public:
      // Type codes for telemetry entry types
      enum class EntryType : uint8_t {
        Unknown = 0,
        Int = 1,
        Double = 2,
        String = 3,
        Bool = 4,
        AnyTypeAllowed = 5
      };

      class TelemetryMessageEntry{
        public:
          std::string key;
          virtual ~TelemetryMessageEntry() = default;
          virtual EntryType type() const = 0;
      };

      class TelemetryMessageEntryInt : public TelemetryMessageEntry{
        public:
          int32_t value{0};
          EntryType type() const override { return EntryType::Int; }
      };
      class TelemetryMessageEntryFloat : public TelemetryMessageEntry{
        public:
          double value{0.0};
          EntryType type() const override { return EntryType::Double; }
      };
      class TelemetryMessageEntryString : public TelemetryMessageEntry{
        public:
          std::string value;
          EntryType type() const override { return EntryType::String; }
      };
      class TelemetryMessageEntryBool : public TelemetryMessageEntry{
        public:
          bool value{false};
          EntryType type() const override { return EntryType::Bool; }
      };
      class TelemetryMessageEntryAnyTypeAllowed : public TelemetryMessageEntry{
        public:
          std::string value;
          std::string value_type; // required to restore type
          EntryType type() const override { return EntryType::AnyTypeAllowed; }
      };

      std::map<std::string, std::unique_ptr<TelemetryMessageEntry>> entries;
    };

  // This class provides an interface to build telemetry messages with different types of entries and serialize them for transmission and rebuild the messages from serialized data.
  // Currently, message building and serialization is the focus, later on methods to interact with received telemetry for example for display purposes are to be added to this, or the telemetry message directly.
  class TelemetryMessageBuilder
  {
    private:
      TelemetryMessage message_;

    public:

      TelemetryMessageBuilder() = default;
      ~TelemetryMessageBuilder() = default;

      void addIntEntry(const std::string& key, int32_t value);
      void addFloatEntry(const std::string& key, double value);
      void addStringEntry(const std::string& key, const std::string& value);
      void addBoolEntry(const std::string& key, bool value);
      void addAnyTypeAllowedEntry(const std::string& key, const std::string& value, const std::string& value_type);

      std::vector<uint8_t> getSerializedMessage() const;

      bool messageEmpty() const { return message_.entries.empty(); }

      bool resetMessage();

      const TelemetryMessage& getMessage() const { return message_; }

      bool fromSerializedMessage(const std::vector<uint8_t>& data);

  };

}