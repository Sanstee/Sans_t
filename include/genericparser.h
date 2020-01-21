#ifndef __GENERICPARSER_H__
#define __GENERICPARSER_H__
#include "gcc-plugin.h"

#include "blockstorage.h"

#include <stack>

typedef int decl_uid;
//tree then, tree else, tree after, std::size_t cond_idx
typedef std::tuple<tree, tree, tree, std::size_t> cond_tuple;

class GenericParser {
  std::stack<tree> stmt_stack;
  std::stack<cond_tuple> cond_stmt_checkpoints;
  std::vector<std::vector<decl_uid>> active_variables;
  std::stack<tree> scope_end_checkpoints;
  struct expr_filter;
  const std::unordered_set<tree_code> static basic_composite_expr;
  const std::unordered_set<tree_code> static relational_operators;
  const std::unordered_set<tree_code> static logical_operators;
  void push_stmt_list_on_stack(const tree stmt_list);
  std::vector<decl_uid> decl_tree_to_uids(const tree decl);
  void split_bind_expr(const tree bind_expr, tree *const vars,
                       tree *const body);
  void categorize_operand(const tree operand, const expr_filter &filter,
                          std::stack<tree> &redo, std::vector<tree> &keep);
  std::vector<tree> flatten_expr(const tree root_expr,
                                 const expr_filter &filter);
  const expr_filter static cond_expr_filter;
  bool split_decl_expr(const tree decl_expr, tree *const lvalue,
                       tree *const rvalue);
  void split_modify_expr(const tree modify_expr, tree *const lvalue,
                         tree *const rvalue);
  var_influence detach_expr(const tree lhs, const tree rhs);
  int max_precision(const std::vector<tree> &vars);
  bool has_precision_diff(tree_code relational_operator_code,
                          const std::vector<tree> &lhs_vars,
                          const std::vector<tree> &rhs_vars);
  bin_cmp_vec detach_condition(const tree cond);
  tree ref_index_tree(const tree ref);
  std::vector<std::pair<tree, std::unordered_set<decl_uid>>>
  detach_references(const tree expr);
  std::vector<std::pair<tree, std::unordered_set<decl_uid>>>
  detach_modify_expr_refs(const tree modify_expr);
  std::vector<std::pair<tree, std::unordered_set<decl_uid>>>
  detach_decl_expr_refs(const tree decl_expr);
  std::vector<std::pair<tree, std::unordered_set<decl_uid>>>
  detach_decl_or_modify_refs(const tree t);
  var_influence detach_modify_expr(const tree modify_expr);
  var_influence detach_decl_expr(const tree decl_expr);
  var_influence detach_decl_or_modify(const tree t);
  void split_cond_expr(const tree cond_expr, tree *const then_value,
                       tree *const else_value);
  tree peek_branch_head(const tree branch);
  tree peek_branch_tail(const tree branch);
  void push_cond_stmt_checkpoints(const tree cond_expr, const std::size_t idx);
  void push_bind_expr_on_stack(const tree bind_expr);
  void parm_decl_to_stack(const tree function_decl);

public:
  BlockStorage traverse_stmt_tree(const tree function_decl);
  std::unordered_set<decl_uid>
  flag_decl_args(const tree function_decl,
                 const unsigned int min_long_int_precision);
  GenericParser();
};

#endif //__GENERICPARSER_H__