#include "sans_t_scoped.h"
#include "blockinterpreter.h"
#include "genericparser.h"

#include "print-tree.h"
#include "tree-iterator.h"
#include "tree.h"
#include <cassert>
#include <iostream>

#define MIN_LONG_INT_PRECISION 33

/**
 * @brief Whenever a function is parsed into GENERIC, parse it into basic blocks
 * and analyze it for the loop counter variable range mismatch.
 *
 * @param event_data The event data, in this case the GENERIC function tree.
 * @param user_data
 */
static void plugin_finish_parse_function(void *event_data, void *user_data) {
  (void)user_data;
  tree function_decl = (tree)event_data;
  std::vector<std::vector<tree>> code_blocks;
  GenericParser gp;
  std::unordered_set<decl_uid> parm_decl_flags =
      gp.flag_decl_args(function_decl, MIN_LONG_INT_PRECISION);
  if (!parm_decl_flags.empty()) {
    BlockStorage bs = gp.traverse_stmt_tree(function_decl);
    BlockInterpreter bi(bs);

    bi.gen_and_check_model(parm_decl_flags);
  }
}

/**
 * @brief Function registering the callback for GCC. More info at:
 * https://gcc.gnu.org/onlinedocs/gccint/Plugins.html#Plugins
 *
 * @param plugin_info
 * @param version
 * @return int
 */
int plugin_init(struct plugin_name_args *plugin_info,
                struct plugin_gcc_version *version) {
  // mandatory version check.
  if (!plugin_default_version_check(version, &gcc_version)) {
    return 1;
  }

  // A plugin should give some information to the user about itself.
  struct plugin_info pi = {"0.0", "Help info."};
  register_callback(plugin_info->base_name, PLUGIN_INFO, NULL, &pi);

  // The callback for every FUNCTION_DECL.
  register_callback(plugin_info->base_name, PLUGIN_FINISH_PARSE_FUNCTION,
                    plugin_finish_parse_function, NULL);

  return 0;
}