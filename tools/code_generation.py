"""
IDL to Fast DDS Header Generator

This script parses IDL files and generates corresponding Fast DDS C++ header files
following the pattern used by fastddsgen.
"""

import re
import sys
import glob
from pathlib import Path
from typing import List, Dict, Tuple, Optional


class IDLField:
    """Represents a field in an IDL struct"""
    def __init__(self, field_type: str, name: str, default_value: Optional[str] = None):
        self.field_type = field_type
        self.name = name
        self.default_value = default_value
        self.is_primitive = field_type in ['double', 'float', 'int32', 'uint32', 'int64', 'uint64', 
                                            'int16', 'uint16', 'int8', 'uint8', 'char', 'boolean']
        self.is_string = field_type == 'string'
        self.is_custom_type = not self.is_primitive and not self.is_string


class IDLStruct:
    """Represents an IDL struct definition"""
    def __init__(self, namespace: str, name: str, fields: List[IDLField], includes: List[str]):
        self.namespace = namespace  # e.g., "geometry_msgs::msg"
        self.name = name
        self.fields = fields
        self.includes = includes
        
    def get_namespace_parts(self) -> List[str]:
        """Returns namespace parts as a list, e.g., ['geometry_msgs', 'msg']"""
        return self.namespace.split('::')


class IDLParser:
    """Parses IDL files to extract struct definitions"""
    
    @staticmethod
    def parse_file(idl_path: str) -> IDLStruct:
        """Parse an IDL file and return the struct definition"""
        with open(idl_path, 'r') as f:
            content = f.read()
        
        # Extract includes
        includes = []
        include_pattern = r'#include\s+"([^"]+)"'
        for match in re.finditer(include_pattern, content):
            include_file = match.group(1)
            # Convert IDL include to header include
            # e.g., "geometry_msgs/msg/Point.idl" -> "Point.h"
            header_name = Path(include_file).stem + '.h'
            includes.append(header_name)
        
        # Extract module/namespace
        module_pattern = r'module\s+(\w+)\s*{'
        modules = re.findall(module_pattern, content)
        namespace = '::'.join(modules)
        
        # Extract struct name
        struct_pattern = r'struct\s+(\w+)\s*{'
        struct_match = re.search(struct_pattern, content)
        if not struct_match:
            raise ValueError("No struct definition found in IDL file")
        struct_name = struct_match.group(1)
        
        # Extract fields
        fields = []
        # Pattern to match field definitions with optional @default annotation
        field_pattern = r'(?:@default\s*\(\s*value\s*=\s*([^\)]+)\)\s*)?(\w+(?:::\w+)*)\s+(\w+)\s*;'
        
        for match in re.finditer(field_pattern, content):
            default_value = match.group(1)
            field_type = match.group(2)
            field_name = match.group(3)
            
            # Remove quotes from default value if present
            if default_value:
                default_value = default_value.strip().strip('"\'')
            
            fields.append(IDLField(field_type, field_name, default_value))
        
        return IDLStruct(namespace, struct_name, fields, includes)


class HeaderGenerator:
    """Generates Fast DDS C++ header files from IDL structs"""
    
    COPYRIGHT = """// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
// limitations under the License."""
    
    def __init__(self, idl_struct: IDLStruct):
        self.struct = idl_struct
        
    def generate(self) -> str:
        """Generate the complete header file content"""
        parts = [
            self._generate_copyright(),
            self._generate_file_header(),
            self._generate_include_guard_start(),
            self._generate_standard_includes(),
            self._generate_fastdds_includes(),
            self._generate_version_check(),
            self._generate_custom_includes(),
            self._generate_dll_export_macros(),
            self._generate_forward_declarations(),
            self._generate_namespace_start(),
            self._generate_class_definition(),
            self._generate_namespace_end(),
            self._generate_include_guard_end()
        ]
        
        return '\n'.join(parts)
    
    def _generate_copyright(self) -> str:
        return self.COPYRIGHT
    
    def _generate_file_header(self) -> str:
        return f"""
/*!
 * @file {self.struct.name}.h
 * This header file contains the declaration of the described types in the IDL
 * file.
 *
 * This file was generated by the tool fastddsgen.
 */"""
    
    def _generate_include_guard_start(self) -> str:
        namespace_upper = '_'.join(self.struct.get_namespace_parts()).upper()
        return f"""
#ifndef _FAST_DDS_GENERATED_{namespace_upper}_{self.struct.name.upper()}_H_
#define _FAST_DDS_GENERATED_{namespace_upper}_{self.struct.name.upper()}_H_"""
    
    def _generate_standard_includes(self) -> str:
        return """
#include <array>
#include <bitset>
#include <cstdint>
#include <fastcdr/cdr/fixed_size_string.hpp>
#include <fastcdr/xcdr/external.hpp>
#include <fastcdr/xcdr/optional.hpp>
#include <map>
#include <string>
#include <vector>"""
    
    def _generate_fastdds_includes(self) -> str:
        return """
// ------------------------------ Pub Sub Type Start
// ----------------------------
#include <fastdds/rtps/common/InstanceHandle.h>
#include <fastdds/rtps/common/SerializedPayload.h>
#include <fastrtps/utils/md5.h>

#include <fastdds/dds/core/policy/QosPolicies.hpp>
#include <fastdds/dds/topic/TopicDataType.hpp>

#if !defined(GEN_API_VER) || (GEN_API_VER != 2)
#error \\
    Generated {name} is not compatible with current installed Fast DDS. Please, regenerate it with fastddsgen.
#endif  // GEN_API_VER

// ------------------------------ Pub Sub Type End ----------------------------""".format(name=self.struct.name)
    
    def _generate_version_check(self) -> str:
        return ""
    
    def _generate_custom_includes(self) -> str:
        if not self.struct.includes:
            return ""
        
        includes = '\n'.join([f'#include "{inc}"' for inc in self.struct.includes])
        return f"\n{includes}"
    
    def _generate_dll_export_macros(self) -> str:
        struct_upper = self.struct.name.upper()
        return f"""
#if defined(_WIN32)
#if defined(EPROSIMA_USER_DLL_EXPORT)
#define eProsima_user_DllExport __declspec(dllexport)
#else
#define eProsima_user_DllExport
#endif  // EPROSIMA_USER_DLL_EXPORT
#else
#define eProsima_user_DllExport
#endif  // _WIN32

#if defined(_WIN32)
#if defined(EPROSIMA_USER_DLL_EXPORT)
#if defined({struct_upper}_SOURCE)
#define {struct_upper}_DllAPI __declspec(dllexport)
#else
#define {struct_upper}_DllAPI __declspec(dllimport)
#endif  // {struct_upper}_SOURCE
#else
#define {struct_upper}_DllAPI
#endif  // EPROSIMA_USER_DLL_EXPORT
#else
#define {struct_upper}_DllAPI
#endif  // _WIN32"""
    
    def _generate_forward_declarations(self) -> str:
        return """
namespace eprosima {
namespace fastcdr {
class Cdr;
class CdrSizeCalculator;
}  // namespace fastcdr
}  // namespace eprosima"""
    
    def _generate_namespace_start(self) -> str:
        namespaces = self.struct.get_namespace_parts()
        lines = []
        for ns in namespaces:
            lines.append(f"\nnamespace {ns} {{")
        return ''.join(lines)
    
    def _generate_namespace_end(self) -> str:
        namespaces = self.struct.get_namespace_parts()
        lines = []
        for ns in reversed(namespaces):
            lines.append(f"\n}}  // namespace {ns}")
        return ''.join(lines)
    
    def _generate_class_definition(self) -> str:
        full_class_name = f"{self.struct.namespace}::{self.struct.name}"
        
        parts = [
            f"""
/*!
 * @brief This class represents the structure {self.struct.name} defined by the user in the
 * IDL file.
 * @ingroup {self.struct.name}
 */
class {self.struct.name} : public eprosima::fastdds::dds::TopicDataType {{
 public:""",
            self._generate_constructors(full_class_name),
            self._generate_operators(full_class_name),
            self._generate_member_accessors(),
            self._generate_private_members(),
            self._generate_public_type_methods(),
            "\n};"
        ]
        
        return '\n'.join(parts)
    
    def _generate_constructors(self, full_class_name: str) -> str:
        return f"""  /*!
   * @brief Copy constructor.
   * @param x Reference to the object {full_class_name} that will be copied.
   */
  eProsima_user_DllExport {self.struct.name}(const {self.struct.name}& x);

  /*!
   * @brief Move constructor.
   * @param x Reference to the object {full_class_name} that will be copied.
   */
  eProsima_user_DllExport {self.struct.name}({self.struct.name}&& x) noexcept;

  /*!
   * @brief Copy assignment.
   * @param x Reference to the object {full_class_name} that will be copied.
   */
  eProsima_user_DllExport {self.struct.name}& operator=(const {self.struct.name}& x);

  /*!
   * @brief Move assignment.
   * @param x Reference to the object {full_class_name} that will be copied.
   */
  eProsima_user_DllExport {self.struct.name}& operator=({self.struct.name}&& x) noexcept;"""
    
    def _generate_operators(self, full_class_name: str) -> str:
        return f"""
  /*!
   * @brief Comparison operator.
   * @param x {full_class_name} object to compare.
   */
  eProsima_user_DllExport bool operator==(const {self.struct.name}& x) const;

  /*!
   * @brief Comparison operator.
   * @param x {full_class_name} object to compare.
   */
  eProsima_user_DllExport bool operator!=(const {self.struct.name}& x) const;"""
    
    def _generate_member_accessors(self) -> str:
        accessors = []
        
        for field in self.struct.fields:
            if field.is_primitive:
                # Primitive types (double, int, etc.)
                accessors.append(f"""
  /*!
   * @brief This function sets a value in member {field.name}
   * @param _{field.name} New value for member {field.name}
   */
  eProsima_user_DllExport void {field.name}({field.field_type} _{field.name});

  /*!
   * @brief This function returns the value of member {field.name}
   * @return Value of member {field.name}
   */
  eProsima_user_DllExport {field.field_type} {field.name}() const;

  /*!
   * @brief This function returns a reference to member {field.name}
   * @return Reference to member {field.name}
   */
  eProsima_user_DllExport {field.field_type}& {field.name}();""")
            
            elif field.is_string:
                # String type
                accessors.append(f"""
  /*!
   * @brief This function copies the value in member {field.name}
   * @param _{field.name} New value to be copied in member {field.name}
   */
  eProsima_user_DllExport void {field.name}(const std::string& _{field.name});

  /*!
   * @brief This function moves the value in member {field.name}
   * @param _{field.name} New value to be moved in member {field.name}
   */
  eProsima_user_DllExport void {field.name}(std::string&& _{field.name});

  /*!
   * @brief This function returns a constant reference to member {field.name}
   * @return Constant reference to member {field.name}
   */
  eProsima_user_DllExport const std::string& {field.name}() const;

  /*!
   * @brief This function returns a reference to member {field.name}
   * @return Reference to member {field.name}
   */
  eProsima_user_DllExport std::string& {field.name}();""")
            
            else:
                # Custom types
                accessors.append(f"""
  /*!
   * @brief This function copies the value in member {field.name}
   * @param _{field.name} New value to be copied in member {field.name}
   */
  eProsima_user_DllExport void {field.name}(
      const {field.field_type}& _{field.name});

  /*!
   * @brief This function moves the value in member {field.name}
   * @param _{field.name} New value to be moved in member {field.name}
   */
  eProsima_user_DllExport void {field.name}({field.field_type}&& _{field.name});

  /*!
   * @brief This function returns a constant reference to member {field.name}
   * @return Constant reference to member {field.name}
   */
  eProsima_user_DllExport const {field.field_type}& {field.name}() const;

  /*!
   * @brief This function returns a reference to member {field.name}
   * @return Reference to member {field.name}
   */
  eProsima_user_DllExport {field.field_type}& {field.name}();""")
        
        return '\n'.join(accessors)
    
    def _generate_private_members(self) -> str:
        members = []
        members.append("\n private:")
        for field in self.struct.fields:
            if field.is_string:
                members.append(f"  std::string m_{field.name};")
            else:
                members.append(f"  {field.field_type} m_{field.name};")
        
        return '\n'.join(members)
    
    def _is_bounded_type(self) -> bool:
        """Check if the type is bounded (has fixed size)"""
        # A type is bounded if all fields are primitive types or bounded custom types
        for field in self.struct.fields:
            if field.is_string:
                return False  # Strings are unbounded
            # For custom types, we'd need to check recursively, but we assume bounded for now
        return True
    
    def _is_plain_type(self) -> bool:
        """Check if the type is plain (can be memcpy'd)"""
        # A type is plain if all fields are primitive or plain types, no strings
        for field in self.struct.fields:
            if field.is_string:
                return False
        return True
    
    def _generate_public_type_methods(self) -> str:
        is_bounded = self._is_bounded_type()
        is_plain = self._is_plain_type()
        
        bounded_value = "true" if is_bounded else "false"
        plain_value = "true" if is_plain else "false"
        construct_sample_value = "true" if is_plain else "false"
        
        return f"""
 public:
  typedef {self.struct.name} type;

  eProsima_user_DllExport {self.struct.name}();

  eProsima_user_DllExport ~{self.struct.name}() override;

  eProsima_user_DllExport bool serialize(
      void* data,
      eprosima::fastrtps::rtps::SerializedPayload_t* payload) override {{
    return serialize(data, payload,
                     eprosima::fastdds::dds::DEFAULT_DATA_REPRESENTATION);
  }}

  eProsima_user_DllExport bool serialize(
      void* data, eprosima::fastrtps::rtps::SerializedPayload_t* payload,
      eprosima::fastdds::dds::DataRepresentationId_t data_representation)
      override;

  eProsima_user_DllExport bool deserialize(
      eprosima::fastrtps::rtps::SerializedPayload_t* payload,
      void* data) override;

  eProsima_user_DllExport std::function<uint32_t()> getSerializedSizeProvider(
      void* data) override {{
    return getSerializedSizeProvider(
        data, eprosima::fastdds::dds::DEFAULT_DATA_REPRESENTATION);
  }}

  eProsima_user_DllExport std::function<uint32_t()> getSerializedSizeProvider(
      void* data,
      eprosima::fastdds::dds::DataRepresentationId_t data_representation)
      override;

  eProsima_user_DllExport bool getKey(
      void* data, eprosima::fastrtps::rtps::InstanceHandle_t* ihandle,
      bool force_md5 = false) override;

  eProsima_user_DllExport void* createData() override;

  eProsima_user_DllExport void deleteData(void* data) override;

#ifdef TOPIC_DATA_TYPE_API_HAS_IS_BOUNDED
  eProsima_user_DllExport inline bool is_bounded() const override {{
    return {bounded_value};
  }}

#endif  // TOPIC_DATA_TYPE_API_HAS_IS_BOUNDED

#ifdef TOPIC_DATA_TYPE_API_HAS_IS_PLAIN
  eProsima_user_DllExport inline bool is_plain() const override {{
    return {"is_plain(eprosima::fastdds::dds::DEFAULT_DATA_REPRESENTATION)" if is_plain else plain_value};
  }}

  eProsima_user_DllExport inline bool is_plain(
      eprosima::fastdds::dds::DataRepresentationId_t data_representation)
      const override {{
    static_cast<void>(data_representation);
    return {plain_value};
  }}

#endif  // TOPIC_DATA_TYPE_API_HAS_IS_PLAIN

#ifdef TOPIC_DATA_TYPE_API_HAS_CONSTRUCT_SAMPLE
  eProsima_user_DllExport inline bool construct_sample(
      void* memory) const override {{
    {"new (memory) " + self.struct.name + "();" if construct_sample_value == "true" else "static_cast<void>(memory);"}
    return {construct_sample_value};
  }}

#endif  // TOPIC_DATA_TYPE_API_HAS_CONSTRUCT_SAMPLE

  MD5 m_md5;
  unsigned char* m_keyBuffer;"""
    
    def _generate_include_guard_end(self) -> str:
        namespace_upper = '_'.join(self.struct.get_namespace_parts()).upper()
        return f"""
#endif  // _FAST_DDS_GENERATED_{namespace_upper}_{self.struct.name.upper()}_H_"""


def main():
    if len(sys.argv) < 2:
        print("Usage: python idl_to_header.py <input1.idl> [input2.idl] [*.idl] ...")
        print("\nExamples:")
        print("  python idl_to_header.py Point.idl")
        print("  python idl_to_header.py Point.idl Quaternion.idl Pose.idl")
        print("  python idl_to_header.py *.idl")
        print("  python idl_to_header.py geometry_msgs/msg/*.idl")
        sys.exit(1)
    
    # Expand glob patterns
    idl_files = []
    for pattern in sys.argv[1:]:
        matches = glob.glob(pattern)
        if matches:
            idl_files.extend(matches)
        else:
            # If no matches, treat as literal filename
            idl_files.append(pattern)
    
    if not idl_files:
        print("Error: No IDL files found")
        sys.exit(1)
    
    generated_files = []
    errors = []
    
    for idl_file in idl_files:
        if not Path(idl_file).exists():
            errors.append(f"File '{idl_file}' not found")
            continue
        
        # Parse IDL
        try:
            idl_struct = IDLParser.parse_file(idl_file)
        except Exception as e:
            errors.append(f"Error parsing '{idl_file}': {e}")
            continue
        
        # Generate header
        try:
            generator = HeaderGenerator(idl_struct)
            header_content = generator.generate()
            
            # Output to file
            output_file = f"{idl_struct.name}.h"
            with open(output_file, 'w') as f:
                f.write(header_content)
            
            generated_files.append(output_file)
            print(f"✓ Generated: {output_file}")
        except Exception as e:
            errors.append(f"Error generating header for '{idl_file}': {e}")
    
    # Summary
    print(f"\n{'='*60}")
    print(f"Summary: {len(generated_files)} file(s) generated successfully")
    if errors:
        print(f"Errors: {len(errors)}")
        for error in errors:
            print(f"  - {error}")
        sys.exit(1)


if __name__ == "__main__":
    main()