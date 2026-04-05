#include "auto_apms_behavior_codec/telemetry_message_builder.hpp"

#include <cstring>
#include <iostream>

#include "cbor.h"

using namespace auto_apms_behavior_codec;

void TelemetryMessageBuilder::addIntEntry(const std::string & key, int32_t value)
{
  auto e = std::make_unique<TelemetryMessage::TelemetryMessageEntryInt>();
  e->key = key;
  e->value = value;
  message_.entries[key] = std::move(e);
}

void TelemetryMessageBuilder::addFloatEntry(const std::string & key, double value)
{
  auto e = std::make_unique<TelemetryMessage::TelemetryMessageEntryFloat>();
  e->key = key;
  e->value = value;
  message_.entries[key] = std::move(e);
}

void TelemetryMessageBuilder::addStringEntry(const std::string & key, const std::string & value)
{
  auto e = std::make_unique<TelemetryMessage::TelemetryMessageEntryString>();
  e->key = key;
  e->value = value;
  message_.entries[key] = std::move(e);
}

void TelemetryMessageBuilder::addBoolEntry(const std::string & key, bool value)
{
  auto e = std::make_unique<TelemetryMessage::TelemetryMessageEntryBool>();
  e->key = key;
  e->value = value;
  message_.entries[key] = std::move(e);
}

void TelemetryMessageBuilder::addAnyTypeAllowedEntry(
  const std::string & key, const std::string & value, const std::string & value_type)
{
  auto e = std::make_unique<TelemetryMessage::TelemetryMessageEntryAnyTypeAllowed>();
  e->key = key;
  e->value = value;
  e->value_type = value_type;
  message_.entries[key] = std::move(e);
}

std::vector<uint8_t> TelemetryMessageBuilder::getSerializedMessage() const
{
  // Encode as CBOR map: { key: [ type_code(uint), value ] }, exept for AnyTypeAllowed, here { key: [ type_code(uint), [
  // type(string), value(string) ] ] }
  std::vector<uint8_t> buf(4096);
  CborEncoder encoder;
  cbor_encoder_init(&encoder, buf.data(), buf.size(), 0);

  CborEncoder mapEncoder;
  cbor_encoder_create_map(&encoder, &mapEncoder, message_.entries.size());

  for (const auto & kv : message_.entries) {
    const std::string & key = kv.first;
    const auto & entry_ptr = kv.second;
    // encode key
    cbor_encode_text_string(&mapEncoder, key.c_str(), key.size());

    // value is array [type_code, value]
    CborEncoder arr;
    cbor_encoder_create_array(&mapEncoder, &arr, 2);

    // first element: type code
    uint8_t type_code = static_cast<uint8_t>(entry_ptr->type());
    cbor_encode_uint(&arr, type_code);

    // second element: actual value depending on type
    switch (entry_ptr->type()) {
      case TelemetryMessage::EntryType::Int: {
        auto p = static_cast<TelemetryMessage::TelemetryMessageEntryInt *>(entry_ptr.get());
        cbor_encode_int(&arr, p->value);
        break;
      }
      case TelemetryMessage::EntryType::Double: {
        auto p = static_cast<TelemetryMessage::TelemetryMessageEntryFloat *>(entry_ptr.get());
        cbor_encode_double(&arr, p->value);
        break;
      }
      case TelemetryMessage::EntryType::String: {
        auto p = static_cast<TelemetryMessage::TelemetryMessageEntryString *>(entry_ptr.get());
        cbor_encode_text_string(&arr, p->value.c_str(), p->value.size());
        break;
      }
      case TelemetryMessage::EntryType::Bool: {
        auto p = static_cast<TelemetryMessage::TelemetryMessageEntryBool *>(entry_ptr.get());
        cbor_encode_boolean(&arr, p->value);
        break;
      }
      case TelemetryMessage::EntryType::AnyTypeAllowed: {
        auto p = static_cast<TelemetryMessage::TelemetryMessageEntryAnyTypeAllowed *>(entry_ptr.get());
        // encode inner as array [inner_type, inner_value]
        CborEncoder innerArr;
        cbor_encoder_create_array(&arr, &innerArr, 2);
        cbor_encode_text_string(&innerArr, p->value_type.c_str(), p->value_type.size());
        cbor_encode_text_string(&innerArr, p->value.c_str(), p->value.size());
        cbor_encoder_close_container_checked(&arr, &innerArr);
        break;
      }
      default: {
        // unknown -> encode null
        cbor_encode_null(&arr);
        break;
      }
    }

    cbor_encoder_close_container_checked(&mapEncoder, &arr);
  }

  cbor_encoder_close_container_checked(&encoder, &mapEncoder);

  size_t used = cbor_encoder_get_buffer_size(&encoder, buf.data());
  buf.resize(used);
  return buf;
}

bool TelemetryMessageBuilder::resetMessage()
{
  message_.entries.clear();
  return true;
}

bool TelemetryMessageBuilder::fromSerializedMessage(const std::vector<uint8_t> & data)
{
  message_.entries.clear();

  CborParser parser;
  CborValue root;
  if (cbor_parser_init(data.data(), data.size(), 0, &parser, &root) != CborNoError) {
    std::cerr << "CBOR parse init failed" << std::endl;
    return false;
  }

  if (!cbor_value_is_map(&root)) {
    std::cerr << "CBOR root is not a map" << std::endl;
    return false;
  }

  CborValue mapIt;
  if (cbor_value_enter_container(&root, &mapIt) != CborNoError) {
    std::cerr << "Failed to enter CBOR map" << std::endl;
    return false;
  }

  while (!cbor_value_at_end(&mapIt)) {
    // read key
    size_t keyLen = 0;
    if (cbor_value_get_string_length(&mapIt, &keyLen) != CborNoError) return false;
    std::string key;
    key.resize(keyLen + 1);
    if (cbor_value_copy_text_string(&mapIt, &key[0], &keyLen, &mapIt) != CborNoError) return false;
    key.resize(keyLen);

    // value: expect an array [type_code, value]
    if (!cbor_value_is_array(&mapIt)) return false;
    CborValue arrIt;
    if (cbor_value_enter_container(&mapIt, &arrIt) != CborNoError) return false;

    // first element: type code (uint)
    uint64_t utype = 0;
    if (cbor_value_get_uint64(&arrIt, &utype) != CborNoError) return false;
    cbor_value_advance(&arrIt);
    TelemetryMessage::EntryType type_code = static_cast<TelemetryMessage::EntryType>(utype);

    // second element: actual value
    int64_t int_tmp = 0;
    double dbl_tmp = 0.0;
    bool bool_tmp = false;
    std::string str_tmp;
    std::string type, val;

    switch (type_code) {
      case TelemetryMessage::EntryType::Int:
        if (cbor_value_get_int64(&arrIt, &int_tmp) != CborNoError) return false;
        break;
      case TelemetryMessage::EntryType::Double:
        if (cbor_value_get_double(&arrIt, &dbl_tmp) != CborNoError) return false;
        break;
      case TelemetryMessage::EntryType::String: {
        size_t slen = 0;
        if (cbor_value_get_string_length(&arrIt, &slen) != CborNoError) return false;
        str_tmp.resize(slen + 1);
        if (cbor_value_copy_text_string(&arrIt, &str_tmp[0], &slen, &arrIt) != CborNoError) return false;
        str_tmp.resize(slen);
        break;
      }
      case TelemetryMessage::EntryType::Bool:
        if (cbor_value_get_boolean(&arrIt, &bool_tmp) != CborNoError) return false;
        break;
      case TelemetryMessage::EntryType::AnyTypeAllowed: {
        // expect inner array [inner_type, inner_value]
        if (!cbor_value_is_array(&arrIt)) return false;
        CborValue innerIt;
        if (cbor_value_enter_container(&arrIt, &innerIt) != CborNoError) return false;
        // get type
        size_t itlen = 0;
        if (cbor_value_get_string_length(&innerIt, &itlen) != CborNoError) return false;
        type.resize(itlen + 1);
        if (cbor_value_copy_text_string(&innerIt, &type[0], &itlen, &innerIt) != CborNoError) return false;
        type.resize(itlen);
        // get value
        size_t ivlen = 0;
        if (cbor_value_get_string_length(&innerIt, &ivlen) != CborNoError) return false;
        val.resize(ivlen + 1);
        if (cbor_value_copy_text_string(&innerIt, &val[0], &ivlen, &innerIt) != CborNoError) return false;
        val.resize(ivlen);
        if (cbor_value_leave_container(&arrIt, &innerIt) != CborNoError) return false;
        break;
      }
      default:
        // skip or set empty
        break;
    }

    // construct entry
    switch (type_code) {
      case TelemetryMessage::EntryType::Int: {
        auto e = std::make_unique<TelemetryMessage::TelemetryMessageEntryInt>();
        e->key = key;
        e->value = static_cast<int32_t>(int_tmp);
        message_.entries[key] = std::move(e);
        break;
      }
      case TelemetryMessage::EntryType::Double: {
        auto e = std::make_unique<TelemetryMessage::TelemetryMessageEntryFloat>();
        e->key = key;
        e->value = dbl_tmp;
        message_.entries[key] = std::move(e);
        break;
      }
      case TelemetryMessage::EntryType::String: {
        auto e = std::make_unique<TelemetryMessage::TelemetryMessageEntryString>();
        e->key = key;
        e->value = str_tmp;
        message_.entries[key] = std::move(e);
        break;
      }
      case TelemetryMessage::EntryType::Bool: {
        auto e = std::make_unique<TelemetryMessage::TelemetryMessageEntryBool>();
        e->key = key;
        e->value = bool_tmp;
        message_.entries[key] = std::move(e);
        break;
      }
      case TelemetryMessage::EntryType::AnyTypeAllowed: {
        auto e = std::make_unique<TelemetryMessage::TelemetryMessageEntryAnyTypeAllowed>();
        e->key = key;
        e->value_type = type;
        e->value = val;
        message_.entries[key] = std::move(e);
        break;
      }
      default: {
        auto e = std::make_unique<TelemetryMessage::TelemetryMessageEntryString>();
        e->key = key;
        e->value = "";
        message_.entries[key] = std::move(e);
        break;
      }
    }

    // leave array
    if (cbor_value_leave_container(&mapIt, &arrIt) != CborNoError) {
      // ignore
    }
  }

  // leave map
  if (cbor_value_leave_container(&root, &mapIt) != CborNoError) {
    // ignore
  }

  return true;
}
