#include "gcc-plugin.h"
#include "plugin-version.h"

int plugin_is_GPL_compatible;

static void plugin_finish_parse_function(void *event_data, void *user_data);

int plugin_init(struct plugin_name_args *plugin_info,
                struct plugin_gcc_version *version);