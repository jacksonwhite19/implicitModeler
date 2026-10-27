"""Pure argument construction used by the real private supervisor and its controls."""

def build(args, prefix, retained_arguments):
    output=args.output.resolve()
    command=list(prefix)+['--worker','--case',str(args.case.resolve()),
        '--output',str(output),'--from-layer',str(args.from_layer),'--through-layer',str(args.through_layer),
        '--mode',args.mode]
    from finite_research.partial_stages import choice
    stop=choice(dict(stop_after=getattr(args,'stop_after','complete')))
    if stop!='complete':command+=['--stop-after',stop]
    if getattr(args,'stop_after_transition',False):command+=['--stop-after-transition']
    mode=getattr(args,'prism_producer_mode','coupled')
    if mode not in ('coupled','source_directions'):raise ValueError('UnknownPrismProducerMode')
    if mode=='source_directions':command+=['--prism-producer-mode',mode]
    if type(args.upstream_forecast_diagnostics) is not bool:raise ValueError('ExplicitBooleanUpstreamObserverRequired')
    if args.upstream_forecast_diagnostics:command+=['--upstream-forecast-diagnostics']
    if args.restart is not None:command+=['--restart',str(args.restart.resolve())]
    if getattr(args,'intermediate_meshes',False):command+=['--intermediate-meshes']
    if getattr(args,'seed_query_diagnostics',False):command+=['--seed-query-diagnostics']
    if getattr(args,'carrier_derivative_diagnostics',False):command+=['--carrier-derivative-diagnostics']
    if getattr(args,'surface_carrier',None) is not None:command+=['--surface-carrier',args.surface_carrier]
    if getattr(args,'surface_assurance',None) is not None:command+=['--surface-assurance',args.surface_assurance]
    if getattr(args,'resume_initial',None) is not None:command+=['--resume-initial',str(args.resume_initial.resolve())]
    retained=retained_arguments(args)
    if retained is not None:command+=['--resume-transition',str(retained[0].resolve()),
        '--retained-manifest',str(retained[1].resolve()),'--retained-manifest-sha256',retained[2]]
    if getattr(args,'minimum_free_bytes',None) is not None:command+=['--minimum-free-bytes',str(args.minimum_free_bytes)]
    return command
