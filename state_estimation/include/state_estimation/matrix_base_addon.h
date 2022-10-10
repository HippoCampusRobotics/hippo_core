#pragma once

template <int Width>
void UncorrelateCovariance(Index first) {
  static_assert(ColsAtCompileTime == RowsAtCompileTime, "Only available for square matrices.");
  static_assert(Width <= ColsAtCompileTime);
  assert(first + Width <= ColsAtCompileTime);
  auto diag = derived().template block<Width, Width>(first, first).diagonal();
  UncorrelateCovarianceSetVariance<Width>(first, diag);
}

template <int Width, typename OtherDerived>
void UncorrelateCovarianceSetVariance(Index first,
                                      const MatrixBase<OtherDerived> &other) {
  static_assert(ColsAtCompileTime == RowsAtCompileTime, "Only available for square matrices.");
  static_assert(Width <= ColsAtCompileTime);
  assert(first + Width <= ColsAtCompileTime);
  derived().template block<Width, ColsAtCompileTime>(first, 0).setConstant(Scalar(0));
  derived().template block<ColsAtCompileTime, Width>(0, first).setConstant(Scalar(0));

  int vec_index = 0;
  for (int i = first; i < first + Width; ++i) {
    derived()(i, i) = other(vec_index);
    ++vec_index;
  }
}

template <int Width>
void UncorrelateCovarianceSetVariance(Index first, const Scalar value) {
  static_assert(ColsAtCompileTime == RowsAtCompileTime, "Only available for square matrices.");
  static_assert(Width <= ColsAtCompileTime);
  assert(first + Width <= ColsAtCompileTime);

  derived().template block<Width, ColsAtCompileTime>(first, 0).setConstant(Scalar(0));
  derived().template block<ColsAtCompileTime, Width>(0, first).setConstant(Scalar(0));
  for (int i = first; i < first + Width; ++i) {
    derived()(i, i) = value;
  }
}

template <int Width>
void MakeBlockSymmetric(Index first) {
  static_assert(ColsAtCompileTime == RowsAtCompileTime, "Only available for square matrices.");
  static_assert(Width <= ColsAtCompileTime);
  assert(first + Width <= ColsAtCompileTime);

  if (Width > 1) {
    for (Index row = first + 1; row < first + Width; ++row) {
        for (Index col = first; col < row; ++col) {
            Scalar tmp = (derived()(row, col) + derived()(col, row)) / Scalar(2);
            derived()(row, col) = tmp;
            derived()(col, row) = tmp;
        }
    }
  }
}

template <int Width>
void MakeRowColSymmetric(Index first) {
  static_assert(ColsAtCompileTime == RowsAtCompileTime, "Only available for square matrices.");
  static_assert(Width <= ColsAtCompileTime);
  assert(first + Width <= ColsAtCompileTime);

  MakeBlockSymmetric<Width>(first);
  for (Index row = first; row < first + Width; ++row) {
    for (Index col = 0; col < first; ++col) {
        Scalar tmp = (derived()(row, col) + derived()(col, row)) / Scalar(2);
        derived()(row, col) = derived()(col, row) = tmp;
    }
    for  (Index col = first + Width; col < ColsAtCompileTime; ++col) {
        Scalar tmp = (derived()(row, col) + derived()(col, row)) / Scalar(2);
        derived()(row, col) = derived()(col, row) = tmp;
    }
  }
}
