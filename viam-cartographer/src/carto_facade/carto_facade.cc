// This is an experimental integration of cartographer into RDK.
#include "carto_facade.h"

#include "glog/logging.h"
#include <boost/filesystem.hpp>
#include <boost/dll/runtime_symbol_info.hpp>
#include <exception>

namespace viam {
namespace carto_facade {

std::string std_string_from_bstring(bstring b_str) {
    int len = blength(b_str);
    char *tmp = bstr2cstr(b_str, 0);
    std::string std_str(tmp, len);
    bcstrfree(tmp);
    return std_str;
}

config from_viam_carto_config(viam_carto_config vcc) {
    struct config c;
    for (int i = 0; i < vcc.sensors_len; i++) {
        c.sensors.push_back(std_string_from_bstring(vcc.sensors[i]));
    }

    c.data_dir = std_string_from_bstring(vcc.data_dir);
    c.component_reference = std_string_from_bstring(vcc.component_reference);
    c.map_rate_sec = vcc.map_rate_sec;
    c.mode = vcc.mode;
    c.lidar_config = vcc.lidar_config;
    if (c.sensors.size() == 0) {
      throw  VIAM_CARTO_SENSORS_LIST_EMPTY;
    }
    return c;
};

std::string find_lua_files() {
    std::string configuration_directory;
    auto programLocation = boost::dll::program_location();
    auto relativePathToLuas = programLocation.parent_path().parent_path();
    relativePathToLuas.append("share/cartographer/lua_files");
    boost::filesystem::path absolutePathToLuas(
        "/usr/local/share/cartographer/lua_files");
    if (exists(relativePathToLuas)) {
        VLOG(1) << "Using lua files from relative path";
        configuration_directory = relativePathToLuas.string();
    } else if (exists(absolutePathToLuas)) {
        VLOG(1) << "Using lua files from absolute path";
        configuration_directory = absolutePathToLuas.string();
    } else {
        LOG(ERROR) << "No lua files found, looked in " << relativePathToLuas;
        LOG(ERROR) << "Use 'make install-lua-files' to install lua files into "
                      "/usr/local/share";
    }
    return configuration_directory;
};

CartoFacade::CartoFacade(const viam_carto_config c,
                         const viam_carto_algo_config ac) {
    config = from_viam_carto_config(c);
    algo_config = ac;

    path_to_data = config.data_dir + "/data";
    // TODO: Change to "/internal_state"
    path_to_internal_state = config.data_dir + "/map";
};

int CartoFacade::IOInit() {
  auto cd = find_lua_files();
  if (cd.empty()) {
    return VIAM_CARTO_LUA_CONFIG_NOT_FOUND;
  }
  configuration_directory = cd;
  return VIAM_CARTO_SUCCESS;
};

int CartoFacade::GetPosition(viam_carto_get_position_response *r) {
    bstring cr = bfromcstr("C++ component reference");
    r->x = 100;
    r->y = 200;
    r->z = 300;
    r->o_x = 400;
    r->o_y = 500;
    r->o_z = 600;
    r->imag = 700;
    r->jmag = 800;
    r->kmag = 900;
    r->theta = 1000;
    r->real = 1100;
    r->component_reference = cr;
    return VIAM_CARTO_SUCCESS;
};

int CartoFacade::GetPointCloudMap(viam_carto_get_point_cloud_map_response *r) {
    return VIAM_CARTO_SUCCESS;
};
int CartoFacade::GetInternalState(viam_carto_get_internal_state_response *r) {
    return VIAM_CARTO_SUCCESS;
};

int CartoFacade::Start() { return VIAM_CARTO_SUCCESS; };

int CartoFacade::Stop() { return VIAM_CARTO_SUCCESS; };

int CartoFacade::AddSensorReading(viam_carto_sensor_reading *sr) {
    return VIAM_CARTO_SUCCESS;
};
}  // namespace carto_facade
}  // namespace viam

extern int viam_carto_lib_init(const viam_carto_lib **ppVCL) {
    if (ppVCL == nullptr) {
        /* *errmsg = "viam_carto_lib pointer should not be NULL"; */
        return VIAM_CARTO_VC_INVALID;
    }
    if (!((sizeof(float) == 4) && (CHAR_BIT == 8) && (sizeof(int) == 4))) {
        /* *errmsg = "viam_carto_lib pointer should not be NULL"; */
        return VIAM_CARTO_LIB_PLATFORM_INVALID;
    }

    FLAGS_logtostderr = 1;
    google::InitGoogleLogging("cartographer");

    return VIAM_CARTO_SUCCESS;
};

extern int viam_carto_lib_terminate(const viam_carto_lib **vcl) {
    FLAGS_logtostderr = 0;
    google::ShutdownGoogleLogging();
    return VIAM_CARTO_SUCCESS;
};


extern int viam_carto_init(const viam_carto **ppVC, const viam_carto_config c,
                           const viam_carto_algo_config ac, char **errmsg) {
    if (ppVC == nullptr) {
        *errmsg = "viam_carto pointer should not be NULL";
        return VIAM_CARTO_VC_INVALID;
    }

    // allocate viam_carto struct
    // TODO: Check for nullptr
    viam_carto *vc = (viam_carto *)malloc(sizeof(viam_carto));

    try {
      vc->carto_obj = new viam::carto_facade::CartoFacade(c, ac);
    } catch (int err) {
      free(vc);
      return err;
    } catch (std::exception& e) {
      free(vc);
      LOG(ERROR) << e.what() << "\n";
      return VIAM_CARTO_UNKNOWN_ERROR;
    }

    // point to newly created viam_carto struct
    *ppVC = vc;
    return VIAM_CARTO_SUCCESS;
};

extern int viam_carto_start(const viam_carto **vc, char **errmsg) {
    return VIAM_CARTO_SUCCESS;
};

extern int viam_carto_stop(const viam_carto **vc, char **errmsg) {
    return VIAM_CARTO_SUCCESS;
};

extern int viam_carto_terminate(const viam_carto **ppVC, char **errmsg) {
    viam::carto_facade::CartoFacade *cf =
        static_cast<viam::carto_facade::CartoFacade *>((*ppVC)->carto_obj);
    delete cf;
    free((viam_carto *)*ppVC);
    return VIAM_CARTO_SUCCESS;
};

extern int viam_carto_add_sensor_reading(const viam_carto **vc,
                                         const viam_carto_sensor_reading *sr,
                                         char **errmsg) {
    return VIAM_CARTO_SUCCESS;
};

extern int viam_carto_add_sensor_reading_destroy(viam_carto_sensor_reading *sr,
                                                 char **errmsg) {
    return VIAM_CARTO_SUCCESS;
};

extern int viam_carto_get_position(const viam_carto **vc,
                                   viam_carto_get_position_response *r,
                                   char **errmsg) {
    viam::carto_facade::CartoFacade *cf =
        static_cast<viam::carto_facade::CartoFacade *>((*vc)->carto_obj);
    cf->GetPosition(r);
    return VIAM_CARTO_SUCCESS;
};

extern int viam_carto_get_position_response_destroy(
    viam_carto_get_position_response *r, char **errmsg) {
    int return_code = VIAM_CARTO_SUCCESS;
    int rc = BSTR_OK;
    rc = bdestroy(r->component_reference);
    if (rc != BSTR_OK) {
        // TODO: Write error messages
        return_code = VIAM_CARTO_DESTRUCTOR_ERROR;
    }
    r->component_reference = nullptr;
    return return_code;
};

extern int viam_carto_get_point_cloud_map(
    const viam_carto **vc, viam_carto_get_point_cloud_map_response *r,
    char **errmsg) {
    return VIAM_CARTO_SUCCESS;
};

extern int viam_carto_get_point_cloud_map_response_destroy(
    viam_carto_get_point_cloud_map_response *r, char **errmsg) {
    return VIAM_CARTO_SUCCESS;
};

extern int viam_carto_get_internal_state(
    const viam_carto **vc, viam_carto_get_internal_state_response *r,
    char **errmsg) {
    return VIAM_CARTO_SUCCESS;
};

extern int viam_carto_get_internal_state_response_destroy(
    viam_carto_get_internal_state_response *r, char **errmsg) {
    return VIAM_CARTO_SUCCESS;
};
