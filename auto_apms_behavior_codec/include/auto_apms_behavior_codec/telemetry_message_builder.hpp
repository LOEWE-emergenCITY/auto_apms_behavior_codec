#pragma once

namespace auto_apms_behavior_codec
{
  // Class representation of a telemetry message, work in progress.
  class TelemetryMessage{
    public:
      class TelemetryMessageEntry{
        public:
          std::string key;
      };
      class TelemetryMessageEntryInt : public TelemetryMessageEntry{
        public:
          int32_t value;
      };
      class TelemetryMessageEntryFloat : public TelemetryMessageEntry{
        public:
          double value;
      };
      class TelemetryMessageEntryString : public TelemetryMessageEntry{
        public:
          std::string value;
      };
      class TelemetryMessageEntryBool : public TelemetryMessageEntry{
        public:
          bool value;
      };
      class TelemetryMessageEntryAnyTypeAllowed : public TelemetryMessageEntry{
        public:
          std::string value;
          std::string type;
      };

      std::map<std::string, TelemetryMessageEntry> entries;
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
      void addAnyTypeAllowedEntry(const std::string& key, const std::string& value, const std::string& type);

      std::vector<uint8_t> getSerializedMessage() const;

      bool messageEmpty() const { return message_.entries.empty(); }

      bool resetMessage();

      TelemetryMessage getMessage() const { return message_; }

      bool fromSerializedMessage(const std::vector<uint8_t>& data);

  };

}