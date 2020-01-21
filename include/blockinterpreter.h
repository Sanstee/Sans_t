#ifndef __BLOCKINTERPRETER_H__
#define __BLOCKINTERPRETER_H__

#include "blockstorage.h"

#include <vector>

class BlockInterpreter {
public:
  BlockStorage bs;
  std::vector<std::unordered_set<decl_uid>> cmp_flagged;
  std::vector<std::vector<std::unordered_set<decl_uid>>> state_vec;
  std::vector<std::vector<bool>> state_is_propagated;
  bool add_state(std::size_t idx, std::unordered_set<decl_uid> flagged_vars);
  std::vector<std::unordered_set<decl_uid>>
  get_unpropagated_states(std::size_t idx);
  std::unordered_set<decl_uid>
  gen_outstate_at(std::size_t idx,
                  const std::unordered_set<decl_uid> &flagged_vars,
                  std::size_t local_effect_idx);
  std::unordered_set<decl_uid>
  gen_outstate(std::size_t idx,
               const std::unordered_set<decl_uid> &flagged_vars);
  void add_cmp_flags(std::size_t cond_idx,
                     const std::unordered_set<decl_uid> &flagged);
  void gen_bs_instance(const std::unordered_set<decl_uid> &flagged_vars);
  void gen_cmp_flagged_instance();
  void check_cmp_flagged_instance();
  void report(const tree reference);
  void gen_and_check_model(const std::unordered_set<decl_uid> &flagged_vars);
  BlockInterpreter(BlockStorage bs);
};

#endif //__BLOCKINTERPRETER_H__
