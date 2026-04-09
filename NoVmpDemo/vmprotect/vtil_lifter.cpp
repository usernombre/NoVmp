// Copyright (C) 2020 Can Boluk
// 
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
//
#include "vtil_lifter.hpp"
#include <vector>
#include "subroutines.hpp"
#include "deobfuscator.hpp"
#include "architecture.hpp"
#include "il2vtil.hpp"

#define DISCOVERY_VERBOSE_OUTPUT 1

namespace vmp
{
	vtil::basic_block* lift_il( vtil::basic_block* block, vm_state* vstate )
	{
		// If virtual instruction pointer is not set:
		//
		if ( !vstate->vip )
		{
			// Parse VMENTER:
			//
			auto [entry_stack, entry_vip] = parse_vmenter( vstate, vstate->current_handler_rva );
			vstate->vip = entry_vip;

			// Begin block if none passed.
			//
			if ( !block )
			{
				block = vtil::basic_block::begin( entry_vip );
			}
			// Otherwise, fork the block.
			//
			else
			{
				auto new_block = block->fork( entry_vip );
				// If returned nullptr, it's already explored, skip.
				//
				if ( !new_block )
				{
					std::lock_guard g( block->owner->mutex );
					block = block->owner->explored_blocks[ entry_vip ];
					fassert( block );
					// TODO: Trace possible exits once more ?.
					//
					return block;
				}
				block = new_block;
			}

			// Insert push instructions.
			//
			for ( auto& op : entry_stack )
				block->push( op );

			// Push relocation offset.
			//
			auto treloc = block->tmp( 64 );
			block->mov( treloc, vtil::REG_IMGBASE )
				 ->sub( treloc, vstate->img->get_real_image_base() )
				 ->push( treloc );
		}
		else
		{
			// If passed block is nullptr, it's already explored, skip.
			//
			if ( !block ) return nullptr;
		}

		instruction_stream is;
		while ( 1 )
		{
			// Skip to next instruction and continue parsing the flow linearly
			//
			vtil::vip_t handler_vip = vstate->next();

			if ( !vstate->img->rva_to_section( vstate->current_handler_rva ) )
			{
				// TODO: Whoooops.
				//
				vtil::debug::dump( block->prev[ 0 ] );
				throw std::runtime_error( "Whoooops invalid virtual jump." );
			}

			// Unroll the stream
			//
			is = vstate->unroll();
			instruction_stream is_reduced = is;

			std::vector parameters = extract_parameters( vstate, is_reduced );
			reduce_chunk( vstate, is_reduced, parameters );
			arch::instruction il_instruction = arch::classify( vstate, is_reduced );

			// REMOVE
			vtil::logger::log<CON_GRN>("[HANDLER]\n");
			for ( int i = 0; i < is_reduced.size(); i++ )
			{
				vtil::logger::log<CON_GRN>("%s\n", is_reduced[i].to_string());
			}
			vtil::logger::log<CON_GRN>("HANDLER_OP = %s\n", il_instruction.op);
			for ( int i = 0; i < il_instruction.parameters.size(); i++ )
			{
				vtil::logger::log<CON_GRN>("HANDLER_PARAM[%d] = %p\n", i, il_instruction.parameters[i]);
			}

			// Break out of the loop to handle VM exit
			//
			if ( il_instruction.op == "VMEXIT" ) break;

			// Translate from VMP Arch to VTIL and continue processing
			//
			block->label_begin( handler_vip );
			translate( block, il_instruction );
			block->label_end();
		}

		// Parse VMEXIT to resolve the order registers are popped
		//
		std::vector exit_stack = parse_vmexit( vstate, is );

		// Simulate the VPOP for each register being popped in the routine
		//
		for ( auto& op : exit_stack )
			block->pop( op );

		// Pop target from stack.
		//
		vtil::operand jmp_dest = block->tmp( 64 );
		block->pop( jmp_dest );

		// Insert vexit to the location.
		//
		block->vexit( jmp_dest );
		jmp_dest = block->back().operands[ 0 ];

		return block;
	}
};