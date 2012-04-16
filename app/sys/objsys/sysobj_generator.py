#!/usr/bin/env python
# -*- encoding:utf8 -*-
# TODO
# Вызывать только необходимые функции:
# check_thresholds() и/или report_changes()



import sys

whole_range = None
def IntRange(a,b):
    return 0

types = {16:"uint16_t",8:"uint8_t", 32:"uint32_t", 64:"uint64_t"}
def ctype(attr):
    return types[attr["size"]]

def bytes(attr):
    return attr["size"]/8;

def var_name(attr):
    return attr["name"].lower()

indent = 0

def pri(s=""):
    outstream.write( "    "*indent + s + "\n")


cur_flash_offset = 0

def generate_getter(attrs):
    global indent
    pri('''size_t  ATTR_FUNC( attr_get )( const uint8_t attr_num, void *const to )
{
    void * from = NULL;
    size_t size = 0;
    uint8_t i;

    // сначала обработка общих атрибутов
    size = common_attr_get(&%s, attr_num, to);
    if (size != 0)
        return size;
        
    // иначе обработка атрибутов данного профиля
    for( i=0; i<TOTAL_ATTRIBUTES; i++ ) 
        if( attributes[i].num == attr_num ) {
            from = attributes[i].ptr;
            size = attributes[i].size;
            break;
        }
    if( TOTAL_ATTRIBUTES <= i )
        return 0;
    '''%common_attrs_name)
    indent = 1
    pri("memcopy(to, from, size);")
    pri("return size;")
    indent -= 1
    pri("}")
    pri("REG_ATTR_FUNC( attr_get );")
        


def generate_setter(attrs):
    global indent
    pri( '''size_t  ATTR_FUNC( attr_set )( const uint8_t attr_num, const void *const from )
{
    __label__ __report_change;
    void * to = NULL;
    size_t size = 0;
    uint8_t i;

    // сначала обработка общих атрибутов
    size = common_attr_set(&%s, attr_num, from);
    if (size != 0)
        goto __report_change;
        
    // иначе обработка атрибутов данного профиля
    for( i=0; i<TOTAL_ATTRIBUTES; i++ ) 
        if( attributes[i].num == attr_num ) {
            to = attributes[i].ptr;
            size = attributes[i].size;
            break;
        }
    if( TOTAL_ATTRIBUTES <= i )
        return 0;
    '''%common_attrs_name)
    indent = 1
    pri("umemcopy(to, from, size);")
    indent = 0
    pri("__report_change:")
    indent = 1
    pri("_attr_on_change(OBJOFFSET, &%s, attr_num);"%common_attrs_name)
    pri("return size;")
    indent -= 1
    pri("}")
    pri("REG_ATTR_FUNC( attr_set );")


def generate_size_getters(attrs):
    global indent
    pri('''size_t  ATTR_FUNC( attr_size )( const uint8_t attr_num )
{
    uint8_t i;
    // сначала обработка общих атрибутов
    size_t size = common_attr_size(&%s, attr_num);
    if (size != 0)
        return size;
    
    for( i=0; i<TOTAL_ATTRIBUTES; i++ ) 
        if( attributes[i].num == attr_num )
            return attributes[i].size;

    return 0;
}'''%common_attrs_name)
    indent -= 1
    pri("REG_ATTR_FUNC( attr_size );")

def fill_spaces(s, width):
    return s + " "*max(1, width-len(s))

def generate_vars(attrs):
    global indent 
    indent = 0
    for attr in attrs:
        section = ""
        static = ""
        default = ""
        if "default" in attr:
            default = " = %s"%str(attr["default"])
        if attr.get("flash", False):
            section = ' __attribute__ ((unused,__section__ (".module_infomem" )))'
            static = 'static '
        pri(static + ctype(attr) +" "+var_name(attr) + section + default + ";")

def generate_defines(attrs):
    global indent 
    indent = 0
    for attr in attrs:
        pri("#define " + fill_spaces(attr["name"],30) + "0x%X"%attr["num"])
        pri("#define " + fill_spaces(attr["name"] + "_SIZE",30) + str(bytes(attr)))
        pri("")
        

def create_thresholds(attrs):
    result = []
    for a in attrs:
        result += [a]
        if a["thresholds"] != None:
            activeness = 0
            for t in xrange(len(a["thresholds"])):
                this_active = a["thresholds"][t].get("active", False)
                if type(this_active) in (tuple, list):
                    # (low_bound, high_bound)
                    this_active = (this_active[0] << 1) | this_active[1]
                else: 
                    # should be bool
                    this_active = this_active and 3
                activeness |= this_active << (t*2)
                result += [{
                    "num":a["num"]+6+t*2,
                    "name":a["name"] + "_UPPER_THR_" + a["thresholds"][t]["name"],
                    "size":a["size"],
                    "range":a["range"],
                    "thresholds":None,
                    "network_acces":"rw",
                    "object_access":"",
                    "default":a["thresholds"][t].get("default", (0,0))[1]
                    },
                    {
                    "num":a["num"]+7+t*2,
                    "name":a["name"] + "_LOWER_THR_" + a["thresholds"][t]["name"],
                    "size":a["size"],
                    "range":a["range"],
                    "thresholds":None,
                    "network_acces":"rw",
                    "object_access":"",
                    "default":a["thresholds"][t].get("default", (0,0))[0]
                    },
#                   OFFSETS ARE NOT IMPLEMENTED
#                    {
#                    "num":a["num"]+6+16+t*2,
#                    "name":a["name"] + "_UPGOING_OFFSET_" + a["thresholds"][t]["name"],
#                    "size":a["size"],
#                    "range":a["range"],
#                    "thresholds":None,
#                    "network_acces":"rw",
#                    "object_access":""
#                    },
#                    {
#                    "num":a["num"]+7+16+t*2,
#                    "name":a["name"] + "_DOWNGOING_OFFSET_" + a["thresholds"][t]["name"],
#                    "size":a["size"],
#                    "range":a["range"],
#                    "thresholds":None,
#                    "network_acces":"rw",
#                    "object_access":""
#                    }
                    ]
            result += [
                {
                "num": a["num"]+1,
                "name":a["name"]+"_THR_ACTIVE_LEVELS",
                "size":16,
                "range":whole_range,
                "thresholds":None,
                "network_acces":"rw",
                "object_access":"",
                "default":activeness
                },
                {
                "num":a["num"]+2,
                "name":a["name"]+"_THR_TRIGGERED_LEVELS",
                "size":16,
                "range":whole_range,
                "thresholds":None,
                "network_acces":"rw",
                "object_access":""
                }
                ]
    return result


common_attrs_name = ""
outstream = None

def attrs_cmp(a,b):
    return cmp(a["num"], b["num"])

def generate_astruct(attrs):
    global indent 
    indent = 0
    pri( '''static const struct attributes_t {
    uint8_t     num;
    uint8_t     size;
    void        *ptr;
} attributes[] = {''')
    indent = 1
    for attr in attrs:
        pri("{"+attr["name"]+","+attr["name"]+"_SIZE"+","+"&"+var_name(attr)+"},")
    indent = 0    
    pri("};")
    pri("#define    TOTAL_ATTRIBUTES    (sizeof(attributes)/sizeof(struct attributes_t))")

def generate(attrs, obj):
    global common_attrs_name, outstream;
    a = create_thresholds(attrs)
    a.sort(attrs_cmp)


    base_name = sys.argv[0][:-3] # cut the '.py'
    
    c_file = file(base_name+".c","w")
    h_file = file(base_name+".h","w")
    h_file_attrs = file(base_name+"_attrs.h","w")
    
    outstream = h_file
    pri("#define OBJ " + str(obj))
    pri("#include <%s>"%h_file_attrs.name)
    pri()
    outstream = h_file_attrs
    generate_defines(a)

    outstream = c_file
    pri('''
#include    <_zigzag.h>
#include    <syszig.h>
#include    <infomem.h>
#include    <zzTypes.h>
#include    <%s>
'''%(base_name+".h"))
    pri("#define     OBJOFFSET    onum2offset(OBJ)")
    common_attrs_name = "common_attrs_" + str(obj)
    pri()
    pri('size_t  ATTR_FUNC( attr_get )( const uint8_t attr_num, void *const to );')
    pri('size_t  ATTR_FUNC( attr_set )( const uint8_t attr_num, const void *const from );')
    pri('size_t  ATTR_FUNC( attr_size )( const uint8_t attr_num );')
    pri("common_attr_t %s;"%common_attrs_name)
    pri()
    generate_vars(a)
    pri()
    generate_astruct(a)
    pri()
    generate_getter(a)
    pri()
    generate_setter(a)
    pri()
    generate_size_getters(a)
    pri("""
void init_obj_%d()
{
    common_attrs_init(&%s);
}

INIT(init_obj_%d);
    """%(obj,common_attrs_name,obj))
