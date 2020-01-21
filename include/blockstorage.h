#ifndef __BLOCKSTORAGE_H__
#define __BLOCKSTORAGE_H__
#include "gcc-plugin.h"
#include "tree.h"

#include <unordered_map>
#include <unordered_set>
#include <vector>

typedef int decl_uid;
typedef std::pair<decl_uid, std::unordered_set<decl_uid>> var_influence;
typedef std::unordered_map<decl_uid, std::unordered_set<decl_uid>>
    var_influence_map;
typedef std::pair<std::unordered_set<decl_uid>, std::unordered_set<decl_uid>>
    bin_cmp;
typedef std::vector<bin_cmp> bin_cmp_vec;

typedef std::tuple<tree, std::unordered_set<decl_uid>, std::size_t> ref_tuple;

class BlockStorage {
public:
  std::size_t root_idx;
  std::size_t sink_idx;
  std::size_t block_count;
  std::unordered_map<tree, std::size_t> block_idx;
  std::unordered_map<std::size_t, bin_cmp_vec> comparisons;
  std::vector<std::vector<tree>> code_blocks;
  std::vector<std::vector<var_influence_map>> local_effects;
  std::vector<std::vector<ref_tuple>> ref_idx_vars;
  std::vector<std::vector<std::size_t>> successors;

private:
  var_influence_map initialize_local_effect(
      const std::vector<std::vector<decl_uid>> &in_scope_vars);
  std::size_t initialize_basic_block(
      const tree id_tree,
      const std::vector<std::vector<decl_uid>> &in_scope_vars);
  void link_blocks(const std::size_t pre, const std::size_t suc);

public:
  std::size_t size() const;
  void push_stmt(const std::size_t idx, const tree stmt_tree);
  void add_local_effect(const std::size_t idx,
                        const std::vector<decl_uid> &in_scope_vars);
  void remove_local_effects(const std::size_t idx,
                            const std::vector<decl_uid> &outdated_vars);
  std::size_t get_unreachable();
  std::size_t
  get_successor(const std::size_t pre, const tree id_tree,
                const std::vector<std::vector<decl_uid>> &in_scope_vars);
  std::size_t
  get_cond_successor(const std::size_t pre, const tree id_tree,
                     const std::vector<std::vector<decl_uid>> &in_scope_vars,
                     const bin_cmp_vec bcv);
  bool is_comparison(const std::size_t idx) const;
  void add_ref(const std::size_t idx,
               const std::vector<std::pair<tree, std::unordered_set<decl_uid>>>
                   &ref_idx_influence);
  void update_effect(const std::size_t idx, const var_influence &inf);
  BlockStorage(const tree root_tree,
               const std::vector<std::vector<decl_uid>> &parm_vars);
};

#endif /*__BLOCKSTORAGE_H__*/