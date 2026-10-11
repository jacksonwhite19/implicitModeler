#pragma once
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

using CanonicalK = CGAL::Exact_predicates_inexact_constructions_kernel;
// One synchronous channel owned by the parent, which owns the kernel handle.
// The same callable is installed in CGAL in both diagnostic and extraction modes.
struct CanonicalKernelCallback {
  std::shared_ptr<std::size_t> calls = std::make_shared<std::size_t>(0);
  double operator()(const CanonicalK::Point_3& p) const {
    const double x=CGAL::to_double(p.x()), y=CGAL::to_double(p.y()), z=CGAL::to_double(p.z());
    if (!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(z)) throw std::runtime_error("NonfiniteCoordinate");
    std::cout << std::setprecision(17) << "Q " << x << ' ' << y << ' ' << z << std::endl;
    std::string status;
    if (!(std::cin >> status)) throw std::runtime_error("CallbackChannelClosed");
    if (status != "OK") { std::string detail; std::getline(std::cin,detail); throw std::runtime_error("KernelCallbackRejected"+detail); }
    double value;
    if (!(std::cin >> value) || !std::isfinite(value)) throw std::runtime_error("InvalidCallbackValue");
    ++*calls;
    return value;
  }
};
