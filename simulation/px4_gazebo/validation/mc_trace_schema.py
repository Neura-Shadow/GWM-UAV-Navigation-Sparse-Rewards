"""Read the pinned native POD layouts without converting float32 payload bits."""
import ctypes as C
import re

WORDS = ('sequence begin_wall begin_cpu copy_wall copy_cpu end_wall end_cpu '
         'overhead_wall overhead_cpu generation trajectory_generation flags '
         'state_order output_order events attempts input_sets published begin_hrt end_hrt').split()
TYPES = dict(uint64_t=C.c_uint64, uint32_t=C.c_uint32, uint16_t=C.c_uint16,
             uint8_t=C.c_uint8, int8_t=C.c_int8, int16_t=C.c_int16, int32_t=C.c_int32,
             int64_t=C.c_int64, float=C.c_float, double=C.c_double, bool=C.c_bool)


def native_type(text, name):
    declarations = re.findall(r'^\s*(uint\d+_t|int\d+_t|float|double|bool) (\w+)(?:\[(\d+)\])?;', text, re.M)
    if not declarations: raise ValueError('No native ABI declarations: '+name)
    fields = [(key, TYPES[kind]*int(count) if count else TYPES[kind]) for kind,key,count in declarations]
    return type(name, (C.LittleEndianStructure,), {'_fields_': fields})


def layouts(headers):
    position = native_type(headers['vehicle_local_position.h'], 'Position')
    trajectory = native_type(headers['trajectory_setpoint.h'], 'Trajectory')
    output = native_type(headers['vehicle_local_position_setpoint.h'], 'Output')
    class Record(C.LittleEndianStructure):
        _fields_ = [('word', C.c_uint64*len(WORDS)), ('input', position),
                    ('supplied', trajectory*3), ('attempt_clock', (C.c_uint64*4)*3),
                    ('attempt_order', (C.c_uint64*3)*3), ('attempt_yaw', (C.c_float*4)*3),
                    ('attempt_info', (C.c_uint32*3)*3), ('output', output), ('state_yaw', C.c_float)]
    class Header(C.LittleEndianStructure):
        _fields_ = [('word', C.c_uint64*16), ('run_id', C.c_char*128), ('overlay_id', C.c_char*65)]
    return Header, Record


def schema(headers):
    header, record = layouts(headers)
    return dict(header_size=C.sizeof(header), record_size=C.sizeof(record),
                offsets={key:getattr(record,key).offset for key,_ in record._fields_},
                word_names=WORDS,
                native_sizes={key:C.sizeof(kind) for key,kind in record._fields_ if key in ('input','output')},
                byte_order='little', float_encoding='IEEE754-binary32-raw-payload')
