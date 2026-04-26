#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace auto_apms_behavior_codec
{

/**
 * @brief Container for a telemetry message consisting of named, typed entries.
 *
 * Each entry is stored as a polymorphic TelemetryMessageEntry keyed by name.
 * Supported entry types are: Int, Double, String, Bool, AnyTypeAllowed, and StringArray.
 */
class TelemetryMessage
{
public:
  /// Numeric type codes written into the CBOR stream so the decoder can reconstruct the correct entry type.
  enum class EntryType : uint8_t
  {
    Unknown = 0,
    Int = 1,
    Double = 2,
    String = 3,
    Bool = 4,
    AnyTypeAllowed = 5,
    StringArray = 6
  };

  /// Base class for a single named telemetry entry.
  class TelemetryMessageEntry
  {
  public:
    std::string key;
    virtual ~TelemetryMessageEntry() = default;
    virtual EntryType type() const = 0;
  };

  /// Integer entry (int32_t).
  class TelemetryMessageEntryInt : public TelemetryMessageEntry
  {
  public:
    int32_t value{0};
    EntryType type() const override { return EntryType::Int; }
  };

  /// Floating-point entry (double).
  class TelemetryMessageEntryFloat : public TelemetryMessageEntry
  {
  public:
    double value{0.0};
    EntryType type() const override { return EntryType::Double; }
  };

  /// String entry.
  class TelemetryMessageEntryString : public TelemetryMessageEntry
  {
  public:
    std::string value;
    EntryType type() const override { return EntryType::String; }
  };

  /// Boolean entry.
  class TelemetryMessageEntryBool : public TelemetryMessageEntry
  {
  public:
    bool value{false};
    EntryType type() const override { return EntryType::Bool; }
  };

  /// Entry that can hold any BehaviorTree.CPP port type, stored as a string with an accompanying type tag.
  class TelemetryMessageEntryAnyTypeAllowed : public TelemetryMessageEntry
  {
  public:
    std::string value;
    std::string value_type;  ///< Original type name, required to restore the concrete type.
    EntryType type() const override { return EntryType::AnyTypeAllowed; }
  };

  /// Array-of-strings entry.
  class TelemetryMessageEntryStringArray : public TelemetryMessageEntry
  {
  public:
    std::vector<std::string> value;
    EntryType type() const override { return EntryType::StringArray; }
  };

  std::map<std::string, std::unique_ptr<TelemetryMessageEntry>> entries;
};

/**
 * @brief Builder for constructing, serializing, and deserializing TelemetryMessage instances.
 *
 * Provides a fluent interface to add typed entries and serialize the resulting message to a
 * compact CBOR byte vector. The same class can also reconstruct a TelemetryMessage from
 * previously serialized bytes.
 */
class TelemetryMessageBuilder
{
private:
  TelemetryMessage message_;

public:
  TelemetryMessageBuilder() = default;
  ~TelemetryMessageBuilder() = default;

  /// @name Entry addition
  /// @{
  void addIntEntry(const std::string & key, int32_t value);
  void addFloatEntry(const std::string & key, double value);
  void addStringEntry(const std::string & key, const std::string & value);
  void addBoolEntry(const std::string & key, bool value);
  void addAnyTypeAllowedEntry(const std::string & key, const std::string & value, const std::string & value_type);
  void addStringArrayEntry(const std::string & key, const std::vector<std::string> & value);
  /// @}

  /// Serialize the current message to a CBOR byte vector.
  std::vector<uint8_t> getSerializedMessage() const;

  /// @return true if no entries have been added.
  bool messageEmpty() const { return message_.entries.empty(); }

  /// Clear all entries.
  bool resetMessage();

  /// @return Read-only reference to the underlying TelemetryMessage.
  const TelemetryMessage & getMessage() const { return message_; }

  /// Reconstruct the message from a CBOR byte vector.
  /// @return true on success, false if deserialization fails.
  bool fromSerializedMessage(const std::vector<uint8_t> & data);
};

}  // namespace auto_apms_behavior_codec